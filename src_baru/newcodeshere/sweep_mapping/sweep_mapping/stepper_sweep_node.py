"""Stepper node: petakan 180 derajat, lalu putar cepat 180 derajat untuk pulang.

Kenapa 180, bukan 360:
  Bidang scan RPLIDAR C1 sudah 360 derajat, dan sumbu putar stepper terletak DI
  bidang itu. Memutar bidang tersebut 180 derajat sudah menyapu seluruh bola.
  180 derajat berikutnya hanya menyapu ruang yang SAMA dari sisi sebaliknya -
  selisih kalibrasi sekecil apa pun (sumbu tidak pas, offset sudut, backlash)
  membuat kedua salinan itu tidak berimpit, dan objek muncul dobel.

Satu sweep sekarang terdiri dari dua fase:
  1. MAPPING  (0 -> 180 derajat) dengan kecepatan `rpm`/`delay`, sama seperti dulu.
     `rpm` tetap berarti putaran per menit piringan LiDAR, jadi fase ini makan
     30/rpm detik (separuh dari waktu satu sweep lama).
  2. PULANG   (180 -> 360 derajat) searah, secepat `return_rpm`, TANPA dipetakan.
     Kecepatan dinaikkan dan diturunkan bertahap (`return_ramp_steps`) supaya
     motor tidak kaget dan belt tidak slip.

Node mapping tahu fase mana yang sedang berjalan lewat /stepper/mapping_active
(Bool, latched): true selama fase MAPPING, false selama fase PULANG.

Lainnya sama dengan stepper_sweep_node1.py (versi lama, 360 derajat dipetakan):
  * `num_sweeps`: berhenti otomatis setelah N sweep (0 = muter terus).
  * Jumlah step eksak, LiDAR berhenti tepat di posisi awal (0 derajat).
  * /stepper/sweep_count naik saat fase MAPPING selesai (cloud sudah lengkap),
    /stepper/sweep_done true saat sweep terakhir sudah pulang ke 0 derajat.
  * Ctrl+C di tengah sweep -> motor muter balik lewat jalur terpendek ke posisi awal.
"""

import math
import threading
import time

import lgpio
import rclpy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Int32

from sweep_mapping.cloud_utils import latched_qos

STEP_PIN = 17
DIR_PIN = 27
EN_PIN = 22
GPIOCHIP = 4  # Raspberry Pi 5

# Batas bawah delay per step. pulse() memakai dua time.sleep(delay/2), dan di
# bawah ~1 ms per sleep, Python di Linux non-realtime tidak lagi akurat: motor
# mulai kehilangan step diam-diam sehingga sudut yang dipublish tidak lagi
# cocok dengan posisi fisik LiDAR. Lebih baik dibatasi dan diteriaki.
MIN_STEP_DELAY = 0.002

# Keluaran RPLIDAR C1 untuk memperkirakan kerapatan titik. Angka ini dibaca
# langsung dari log driver pada 3 Agustus 2026:
#   "scan mode: DenseBoost, sample rate: 5 Khz, scan frequency: 12.0 Hz"
# Kalau scan_mode diubah, angka ini ikut berubah dan perkiraannya meleset.
LIDAR_POINTS_PER_SEC = 5000.0


class StepperSweepNode(Node):

    def __init__(self):
        super().__init__('stepper_sweep_node')

        self.h = lgpio.gpiochip_open(GPIOCHIP)
        lgpio.gpio_claim_output(self.h, STEP_PIN)
        lgpio.gpio_claim_output(self.h, DIR_PIN)
        lgpio.gpio_claim_output(self.h, EN_PIN)
        lgpio.gpio_write(self.h, EN_PIN, 0)

        self.declare_parameter('steps_per_rev', 1600)
        # Detik per step motor selama fase MAPPING. Cara lama, tetap didukung
        # penuh supaya perintah yang sudah biasa dipakai (delay:=0.009375)
        # berarti kecepatan yang persis sama.
        self.declare_parameter('delay', 0.018)
        # Kecepatan putar piringan LiDAR selama fase MAPPING, dalam putaran per
        # menit. Dipakai HANYA kalau diisi > 0; kalau tidak, `delay` yang menentukan.
        self.declare_parameter('rpm', 0.0)
        # Kecepatan fase PULANG (putaran per menit piringan LiDAR). Dikunci di
        # batas MIN_STEP_DELAY kalau terlalu cepat, dan tidak pernah lebih
        # lambat dari fase MAPPING.
        self.declare_parameter('return_rpm', 9.0)
        # Jumlah step untuk menaikkan kecepatan dari `rpm` ke `return_rpm` di
        # awal fase PULANG, dan menurunkannya lagi di akhir. 0 = langsung loncat.
        self.declare_parameter('return_ramp_steps', 200)
        self.declare_parameter('direction', 1)
        # Motor pulley (30T) / LiDAR pulley (60T) -> LiDAR muter separuh kecepatan motor
        self.declare_parameter('gear_ratio', 30.0 / 60.0)
        # Jumlah sweep (1 sweep = 180 derajat dipetakan + 180 derajat pulang).
        # 0 = muter terus tanpa henti.
        self.declare_parameter('num_sweeps', 1)
        # Kembali ke posisi awal kalau di-Ctrl+C sebelum semua sweep selesai.
        self.declare_parameter('return_home_on_abort', True)

        self.steps_per_rev = int(self.get_parameter('steps_per_rev').value)
        self.direction = int(self.get_parameter('direction').value)
        self.gear_ratio = float(self.get_parameter('gear_ratio').value)
        self.num_sweeps = int(self.get_parameter('num_sweeps').value)
        self.return_home_on_abort = bool(
            self.get_parameter('return_home_on_abort').value
        )
        self.return_ramp_steps = max(
            0, int(self.get_parameter('return_ramp_steps').value))

        lgpio.gpio_write(self.h, DIR_PIN, 1 if self.direction == 1 else 0)

        if self.gear_ratio <= 0.0:
            raise ValueError('gear_ratio harus lebih besar dari 0')

        # Berapa step motor untuk membuat LiDAR muter 360 derajat penuh.
        # steps_per_rev=1600, gear_ratio=0.5 -> 3200 step per sweep.
        self.steps_per_sweep = int(round(self.steps_per_rev / self.gear_ratio))
        # 180 derajat pertama dipetakan, sisanya pulang. Sisa dihitung dari
        # pengurangan (bukan dibagi dua lagi) supaya total tetap eksak 360 derajat
        # walau steps_per_sweep ganjil.
        self.mapping_steps = self.steps_per_sweep // 2
        self.return_steps = self.steps_per_sweep - self.mapping_steps

        self.delay = self.tentukan_delay()
        self.return_delay = min(
            self.delay_from_rpm(
                float(self.get_parameter('return_rpm').value), 'return_rpm'),
            self.delay,
        )

        # Total step yang harus ditempuh. 0 berarti tidak terbatas.
        self.target_steps = (
            self.num_sweeps * self.steps_per_sweep if self.num_sweeps > 0 else 0
        )

        self.current_step = 0   # posisi relatif terhadap titik awal (bertanda)
        self.steps_done = 0     # jumlah step yang sudah ditempuh (selalu positif)
        self.sweeps_done = 0
        self.mapping_active = None
        self.running = True
        self.aborted = False
        self.finished = False

        self.angle_pub = self.create_publisher(Float32, '/stepper/angle', 10)
        self.steps_pub = self.create_publisher(Int32, '/stepper/steps', 10)
        self.status_pub = self.create_publisher(Bool, '/stepper/status', 10)
        self.sweep_count_pub = self.create_publisher(
            Int32, '/stepper/sweep_count', latched_qos()
        )
        self.sweep_done_pub = self.create_publisher(
            Bool, '/stepper/sweep_done', latched_qos()
        )
        self.mapping_active_pub = self.create_publisher(
            Bool, '/stepper/mapping_active', latched_qos()
        )

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.create_subscription(Bool, '/stepper/enable', self.enable_cb, 10)
        self.create_subscription(Bool, '/stepper/direction', self.direction_cb, 10)
        self.create_subscription(Float32, '/stepper/speed', self.speed_cb, 10)

        self.publish_sweep_count()
        # Umumkan state awal "belum selesai". Karena topic ini latched, subscriber
        # yang nyala belakangan (atau nyala ulang) jadi tahu scan masih berjalan,
        # bukan malah membaca nilai `true` sisa sesi sebelumnya.
        self.sweep_done_pub.publish(Bool(data=False))

        mapping_seconds = self.mapping_steps * self.delay
        return_seconds = self.estimate_return_seconds()
        sweep_seconds = mapping_seconds + return_seconds
        points_per_sweep = int(mapping_seconds * LIDAR_POINTS_PER_SEC)

        self.get_logger().info(
            f'Mapping 180 derajat: {self.effective_rpm():.2f} RPM '
            f'({self.delay * 1000:.2f} ms/step, {mapping_seconds:.1f} detik) '
            f'-> kira-kira {points_per_sweep} titik per sweep.'
        )
        self.get_logger().info(
            f'Pulang 180 derajat tanpa mapping: '
            f'{60.0 / (self.return_delay * self.steps_per_sweep):.2f} RPM '
            f'({self.return_delay * 1000:.2f} ms/step, '
            f'ramp {self.return_ramp_steps} step, ~{return_seconds:.1f} detik).'
        )

        if self.target_steps > 0:
            self.get_logger().info(
                f'Target {self.num_sweeps} sweep x {self.steps_per_sweep} step '
                f'= {self.target_steps} step '
                f'(estimasi {self.num_sweeps * sweep_seconds:.0f} detik). '
                'LiDAR akan berhenti tepat di posisi awal.'
            )
        else:
            self.get_logger().info(
                'num_sweeps=0 -> motor muter terus tanpa henti.'
            )
        self.get_logger().warn(
            'Pastikan LiDAR sudah lurus/sejajar SEKARANG. '
            'Posisi saat ini dipakai sebagai titik nol.'
        )

        self.set_mapping_active(True)

        self.motor_thread = threading.Thread(target=self.motor_loop, daemon=True)
        self.motor_thread.start()

    # ==========================================================
    # Kecepatan
    # ==========================================================

    def tentukan_delay(self):
        """Pilih sumber kecepatan MAPPING: `rpm` kalau diisi, kalau tidak `delay`.

        Jalur `delay` sengaja dibiarkan apa adanya - tidak dibatasi, tidak
        diutak-atik - supaya perintah lama seperti `delay:=0.009375`
        berarti kecepatan yang persis sama. Kalau nilainya berbahaya, user
        diberi tahu tapi tetap dituruti.
        """
        rpm = float(self.get_parameter('rpm').value)
        delay = float(self.get_parameter('delay').value)

        if rpm > 0.0:
            return self.delay_from_rpm(rpm, 'rpm')

        if delay <= 0.0:
            raise ValueError('delay harus lebih besar dari 0')

        if delay < MIN_STEP_DELAY:
            self.get_logger().warn(
                f'delay={delay * 1000:.2f} ms/step di bawah batas aman '
                f'{MIN_STEP_DELAY * 1000:.0f} ms. Motor bisa kehilangan step '
                'diam-diam sehingga sudut yang dipublish tidak lagi cocok '
                'dengan posisi fisik LiDAR. Nilainya tetap dipakai.'
            )

        return delay

    def delay_from_rpm(self, rpm, nama):
        """Ubah RPM piringan LiDAR jadi detik per step motor.

        Satu putaran LiDAR = steps_per_sweep step motor, ditempuh dalam
        60/rpm detik. Kalau hasilnya lebih cepat dari yang sanggup dikeluarkan
        time.sleep, delay dikunci di MIN_STEP_DELAY dan user diberi tahu RPM
        efektif yang sebenarnya dipakai - bukan diam-diam meleset.
        """
        if rpm <= 0.0:
            raise ValueError(f'{nama} harus lebih besar dari 0')

        delay = 60.0 / (rpm * self.steps_per_sweep)

        if delay < MIN_STEP_DELAY:
            max_rpm = 60.0 / (MIN_STEP_DELAY * self.steps_per_sweep)
            self.get_logger().warn(
                f'{nama}={rpm:.2f} terlalu cepat untuk step timing berbasis '
                f'time.sleep (butuh delay {delay * 1000:.2f} ms/step). '
                f'Dikunci di {max_rpm:.2f} RPM. Kalau perlu lebih cepat, '
                'turunkan steps_per_rev (microstepping driver).'
            )
            delay = MIN_STEP_DELAY

        return delay

    def effective_rpm(self):
        """RPM fase MAPPING yang benar-benar dipakai setelah pembatasan."""
        return 60.0 / (self.delay * self.steps_per_sweep)

    def return_step_delay(self, i):
        """Delay step ke-i (0-based) dari fase PULANG, dengan ramp naik-turun.

        Yang diinterpolasi linear adalah KECEPATAN (step/detik), bukan delay,
        supaya percepatannya merata. Ujung-ujung ramp bertemu kecepatan
        mapping, jadi tidak ada loncatan kecepatan saat berganti fase.
        """
        ramp = min(self.return_ramp_steps, self.return_steps // 2)
        if ramp == 0:
            return self.return_delay

        if i < ramp:
            frac = (i + 1) / ramp
        elif i >= self.return_steps - ramp:
            frac = (self.return_steps - i) / ramp
        else:
            return self.return_delay

        v_map = 1.0 / self.delay
        v_ret = 1.0 / self.return_delay
        return 1.0 / (v_map + (v_ret - v_map) * frac)

    def estimate_return_seconds(self):
        return sum(self.return_step_delay(i) for i in range(self.return_steps))

    # ==========================================================
    # Motor
    # ==========================================================

    def pulse(self, delay):
        """Satu step motor."""
        lgpio.gpio_write(self.h, STEP_PIN, 1)
        time.sleep(delay / 2.0)
        lgpio.gpio_write(self.h, STEP_PIN, 0)
        time.sleep(delay / 2.0)

    def lidar_angle_rad(self):
        """Sudut fisik LiDAR (rad, 0..2pi) hasil step motor + gear ratio."""
        motor_angle_deg = (self.current_step / self.steps_per_rev) * 360.0
        lidar_angle_deg = (motor_angle_deg * self.gear_ratio) % 360.0
        return math.radians(lidar_angle_deg)

    def set_mapping_active(self, active):
        """Umumkan fase ke node mapping, hanya kalau berubah."""
        if self.mapping_active == active:
            return
        self.mapping_active = active
        self.mapping_active_pub.publish(Bool(data=active))

    def motor_loop(self):
        while rclpy.ok() and not self.aborted and not self.finished:
            if not self.running:
                time.sleep(0.1)
                continue

            # Posisi di dalam sweep yang sedang berjalan: 0..steps_per_sweep-1.
            cycle_step = self.steps_done % self.steps_per_sweep

            if cycle_step == 0:
                # Awal sweep (termasuk sweep ke-2 dst.): mulai memetakan lagi.
                self.set_mapping_active(True)

            if cycle_step < self.mapping_steps:
                delay = self.delay
            else:
                delay = self.return_step_delay(cycle_step - self.mapping_steps)

            self.pulse(delay)

            self.current_step += 1 if self.direction == 1 else -1
            self.steps_done += 1
            cycle_step += 1

            angle_rad = self.lidar_angle_rad()
            stamp = self.get_clock().now().to_msg()

            self.angle_pub.publish(Float32(data=angle_rad))
            self.steps_pub.publish(Int32(data=self.current_step))
            self.status_pub.publish(Bool(data=self.running))
            self.broadcast_tf(angle_rad, stamp)

            if cycle_step == self.mapping_steps:
                # 180 derajat terpetakan. Sudut ini sudah dipublish di atas, jadi
                # node mapping masih punya sudut ujung untuk interpolasi.
                self.set_mapping_active(False)
                self.sweeps_done += 1
                self.publish_sweep_count()
                self.get_logger().info(
                    f'Sweep {self.sweeps_done}'
                    + (f'/{self.num_sweeps}' if self.target_steps > 0 else '')
                    + ' selesai dipetakan (180 derajat). Pulang cepat...'
                )

            if self.target_steps > 0 and self.steps_done >= self.target_steps:
                self.complete_scan()

    def complete_scan(self):
        """Semua sweep selesai dan sudah pulang: matikan motor, umumkan."""
        self.finished = True
        self.running = False
        lgpio.gpio_write(self.h, EN_PIN, 1)

        self.status_pub.publish(Bool(data=False))
        self.sweep_done_pub.publish(Bool(data=True))

        self.get_logger().info(
            f'SCAN COMPLETE - {self.sweeps_done} sweep, '
            f'{self.steps_done} step, LiDAR kembali ke 0.0 derajat. '
            'Motor dimatikan.'
        )

    def publish_sweep_count(self):
        self.sweep_count_pub.publish(Int32(data=self.sweeps_done))

    # ==========================================================
    # Pulang ke posisi awal (Ctrl+C)
    # ==========================================================

    def steps_from_home(self):
        """Sisa step dari posisi sekarang ke titik nol terdekat (0..steps_per_sweep)."""
        return self.steps_done % self.steps_per_sweep

    def return_home(self):
        """Muter balik ke posisi awal lewat jalur terpendek.

        Dipakai kalau user Ctrl+C sebelum semua sweep selesai. Titik yang
        terambil di sini tidak ikut dipetakan karena mapping_active dimatikan dulu.
        """
        self.set_mapping_active(False)

        offset = self.steps_from_home()
        if offset == 0:
            return

        # Jalur terpendek: mundur `offset` step, atau maju sisa sweep.
        backward = offset
        forward = self.steps_per_sweep - offset

        if backward <= forward:
            steps = backward
            move_dir = 0 if self.direction == 1 else 1
            step_delta = -1 if self.direction == 1 else 1
        else:
            steps = forward
            move_dir = 1 if self.direction == 1 else 0
            step_delta = 1 if self.direction == 1 else -1

        self.get_logger().info(
            f'Kembali ke posisi awal: {steps} step '
            f'(estimasi {steps * self.delay:.1f} detik)...'
        )

        lgpio.gpio_write(self.h, EN_PIN, 0)
        lgpio.gpio_write(self.h, DIR_PIN, move_dir)
        time.sleep(0.01)  # beri waktu driver membaca perubahan DIR

        for _ in range(steps):
            self.pulse(self.delay)
            self.current_step += step_delta

        self.get_logger().info(
            f'LiDAR sudah lurus lagi di {math.degrees(self.lidar_angle_rad()):.2f} derajat.'
        )

    def request_stop(self):
        """Hentikan motor loop, lalu (opsional) pulang ke posisi awal."""
        already_stopped = self.aborted or self.finished

        self.aborted = True
        if self.motor_thread.is_alive():
            self.motor_thread.join(timeout=5.0)

        # Sudah selesai normal = sudah di posisi awal, tidak perlu pulang lagi.
        if already_stopped:
            return

        if self.return_home_on_abort:
            try:
                self.return_home()
            except Exception as exc:  # noqa: BLE001 - jangan sampai gagal cleanup GPIO
                self.get_logger().error(f'Gagal kembali ke posisi awal: {exc}')
        else:
            self.get_logger().warn(
                'Motor berhenti di tengah sweep. Luruskan LiDAR manual '
                'sebelum scan berikutnya.'
            )

    # ==========================================================
    # TF & callbacks
    # ==========================================================

    def broadcast_tf(self, angle_rad, stamp):
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'lidar_tilt'

        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.1

        t.transform.rotation.x = math.sin(angle_rad / 2)
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = math.cos(angle_rad / 2)

        self.tf_broadcaster.sendTransform(t)

    def enable_cb(self, msg):
        if self.finished:
            return
        self.running = msg.data
        lgpio.gpio_write(self.h, EN_PIN, 0 if msg.data else 1)

    def direction_cb(self, msg):
        self.direction = 1 if msg.data else 0
        lgpio.gpio_write(self.h, DIR_PIN, self.direction)

    def speed_cb(self, msg):
        """/stepper/speed berisi detik per step fase MAPPING, sama seperti dulu.

        Satuannya sengaja tidak diubah jadi RPM supaya perintah di
        PANDUAN_SISTEM.md 6.g tetap berlaku apa adanya.
        """
        nilai = float(msg.data)
        if nilai <= 0.0:
            self.get_logger().warn(
                f'/stepper/speed={nilai} diabaikan, harus lebih besar dari 0'
            )
            return

        self.delay = nilai
        self.get_logger().info(
            f'Kecepatan mapping diubah ke {nilai * 1000:.2f} ms/step '
            f'({self.effective_rpm():.2f} RPM)'
        )

    def destroy_node(self):
        lgpio.gpio_write(self.h, EN_PIN, 1)
        lgpio.gpiochip_close(self.h)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = StepperSweepNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.request_stop()
    node.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
