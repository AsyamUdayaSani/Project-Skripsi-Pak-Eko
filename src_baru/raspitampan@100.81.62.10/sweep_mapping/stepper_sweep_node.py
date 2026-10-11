"""Stepper node dengan jumlah sweep yang bisa diatur lewat parameter.

Perbedaan dengan stepper_controller/stepper_node.py:
  * Parameter `num_sweeps`: motor berhenti otomatis setelah N sweep (0 = muter terus).
  * Jumlah step dihitung eksak, jadi LiDAR berhenti tepat di posisi awal (0 derajat).
  * Publish progres sweep ke /stepper/sweep_count dan /stepper/sweep_done.
  * Ctrl+C di tengah sweep -> motor muter balik lewat jalur terpendek ke posisi awal.

File asli tidak diubah sama sekali.
"""

import math
import threading
import time

import lgpio
import rclpy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from std_msgs.msg import Bool, Float32, Int32

STEP_PIN = 17
DIR_PIN = 27
EN_PIN = 22
GPIOCHIP = 4  # Raspberry Pi 5


def latched_qos():
    """QoS latched: subscriber yang telat nyala tetap dapat pesan terakhir."""
    return QoSProfile(
        depth=1,
        history=QoSHistoryPolicy.KEEP_LAST,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    )


class StepperSweepNode(Node):

    def __init__(self):
        super().__init__('stepper_sweep_node')

        self.h = lgpio.gpiochip_open(GPIOCHIP)
        lgpio.gpio_claim_output(self.h, STEP_PIN)
        lgpio.gpio_claim_output(self.h, DIR_PIN)
        lgpio.gpio_claim_output(self.h, EN_PIN)
        lgpio.gpio_write(self.h, EN_PIN, 0)

        self.declare_parameter('steps_per_rev', 1600)
        self.declare_parameter('delay', 0.018)
        self.declare_parameter('direction', 1)
        # Motor pulley (30T) / LiDAR pulley (60T) -> LiDAR muter separuh kecepatan motor
        self.declare_parameter('gear_ratio', 30.0 / 60.0)
        # Jumlah sweep (1 sweep = LiDAR muter 360 derajat). 0 = muter terus tanpa henti.
        self.declare_parameter('num_sweeps', 1)
        # Kembali ke posisi awal kalau di-Ctrl+C sebelum semua sweep selesai.
        self.declare_parameter('return_home_on_abort', True)

        self.steps_per_rev = int(self.get_parameter('steps_per_rev').value)
        self.delay = float(self.get_parameter('delay').value)
        self.direction = int(self.get_parameter('direction').value)
        self.gear_ratio = float(self.get_parameter('gear_ratio').value)
        self.num_sweeps = int(self.get_parameter('num_sweeps').value)
        self.return_home_on_abort = bool(
            self.get_parameter('return_home_on_abort').value
        )

        lgpio.gpio_write(self.h, DIR_PIN, 1 if self.direction == 1 else 0)

        if self.gear_ratio <= 0.0:
            raise ValueError('gear_ratio harus lebih besar dari 0')

        # Berapa step motor untuk membuat LiDAR muter 360 derajat penuh.
        # steps_per_rev=1600, gear_ratio=0.5 -> 3200 step per sweep.
        self.steps_per_sweep = int(round(self.steps_per_rev / self.gear_ratio))

        # Total step yang harus ditempuh. 0 berarti tidak terbatas.
        self.target_steps = (
            self.num_sweeps * self.steps_per_sweep if self.num_sweeps > 0 else 0
        )

        self.current_step = 0   # posisi relatif terhadap titik awal (bertanda)
        self.steps_done = 0     # jumlah step yang sudah ditempuh (selalu positif)
        self.sweeps_done = 0
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

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.create_subscription(Bool, '/stepper/enable', self.enable_cb, 10)
        self.create_subscription(Bool, '/stepper/direction', self.direction_cb, 10)
        self.create_subscription(Float32, '/stepper/speed', self.speed_cb, 10)

        self.publish_sweep_count()
        # Umumkan state awal "belum selesai". Karena topic ini latched, subscriber
        # yang nyala belakangan (atau nyala ulang) jadi tahu scan masih berjalan,
        # bukan malah membaca nilai `true` sisa sesi sebelumnya.
        self.sweep_done_pub.publish(Bool(data=False))

        sweep_seconds = self.steps_per_sweep * self.delay
        if self.target_steps > 0:
            self.get_logger().info(
                f'Target {self.num_sweeps} sweep x {self.steps_per_sweep} step '
                f'= {self.target_steps} step '
                f'(estimasi {self.num_sweeps * sweep_seconds:.0f} detik). '
                'LiDAR akan berhenti tepat di posisi awal.'
            )
        else:
            self.get_logger().info(
                'num_sweeps=0 -> motor muter terus tanpa henti (mode lama).'
            )
        self.get_logger().warn(
            'Pastikan LiDAR sudah lurus/sejajar SEKARANG. '
            'Posisi saat ini dipakai sebagai titik nol.'
        )

        self.motor_thread = threading.Thread(target=self.motor_loop, daemon=True)
        self.motor_thread.start()

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

    def motor_loop(self):
        while rclpy.ok() and not self.aborted and not self.finished:
            if not self.running:
                time.sleep(0.1)
                continue

            self.pulse(self.delay)

            self.current_step += 1 if self.direction == 1 else -1
            self.steps_done += 1

            angle_rad = self.lidar_angle_rad()
            stamp = self.get_clock().now().to_msg()

            self.angle_pub.publish(Float32(data=angle_rad))
            self.steps_pub.publish(Int32(data=self.current_step))
            self.status_pub.publish(Bool(data=self.running))
            self.broadcast_tf(angle_rad, stamp)

            # Satu sweep selesai setiap kelipatan steps_per_sweep.
            if self.steps_done % self.steps_per_sweep == 0:
                self.sweeps_done += 1
                self.publish_sweep_count()
                self.get_logger().info(
                    f'Sweep {self.sweeps_done}'
                    + (f'/{self.num_sweeps}' if self.target_steps > 0 else '')
                    + ' selesai'
                )

            if self.target_steps > 0 and self.steps_done >= self.target_steps:
                self.complete_scan()

    def complete_scan(self):
        """Semua sweep selesai: matikan motor, umumkan ke node lain."""
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
    # Pulang ke posisi awal
    # ==========================================================

    def steps_from_home(self):
        """Sisa step dari posisi sekarang ke titik nol terdekat (0..steps_per_sweep)."""
        return self.steps_done % self.steps_per_sweep

    def return_home(self):
        """Muter balik ke posisi awal lewat jalur terpendek.

        Dipakai kalau user Ctrl+C sebelum semua sweep selesai.
        """
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
        self.delay = float(msg.data)

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
