"""Mapping 3D dengan koreksi IMU, untuk rig yang dipasang di drone.

Bedanya dengan sweep_mapping/mapping_3d_sweep.py:
  * Setiap sinar dirotasi dua kali: sudut stepper (seperti biasa) lalu orientasi
    rangka dari IMU. Kalau drone miring atau bergetar, titik tetap jatuh di
    tempat yang benar.
  * Orientasi IMU diinterpolasi PER SINAR, sama seperti sudut stepper.
    mapping_3d_old.py memakai satu orientasi untuk seluruh pesan scan (~450
    sinar, ~100 ms) - untuk rig statis itu cukup, untuk drone yang goyang justru
    jadi sumber blur.
  * Default hanya roll dan pitch yang dikoreksi. Keduanya mengacu ke gravitasi
    sehingga tidak pernah drift. Yaw mengacu ke magnetometer, dan magnetometer
    di dekat motor + ESC drone tidak bisa dipercaya - nyalakan lewat `use_yaw`
    hanya kalau uji lapangan menunjukkan bacaannya bersih.

ASUMSI PENTING: posisi drone dianggap TETAP selama satu sweep. Node ini hanya
mengoreksi orientasi. Kalau drone bergeser, pergeseran itu masuk ke peta sebagai
galat dan tidak ada yang bisa dilakukan node ini untuk memperbaikinya.

Node dan launch file sweep_mapping tidak diubah oleh file ini; stepper dan bag
recorder dipakai langsung dari paket sweep_mapping.
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan, PointCloud2
from std_msgs.msg import Bool, Float32, Int32

from sweep_mapping.cloud_utils import (
    POINT_DTYPE,
    latched_qos,
    make_cloud,
    merge_chunks,
)

# Panjang riwayat orientasi yang disimpan. Modul WIT terbit di ~100-200 Hz, jadi
# 400 sampel = 2-4 detik. Cukup panjang untuk menutupi satu pesan scan beserta
# latensi serial, cukup pendek supaya np.interp tetap murah.
IMU_BUFFER = 400

# Riwayat sudut stepper. Sama dengan mapping_3d_sweep.py.
SERVO_BUFFER = 100


def quat_to_rpy(x, y, z, w):
    """Quaternion -> roll, pitch, yaw (radian), konvensi ZYX Tait-Bryan.

    Konvensi ini sengaja dicocokkan dengan get_quaternion_from_euler() di
    wit_ros2_imu.py:108 supaya hasilnya benar-benar mengembalikan roll/pitch/yaw
    yang tadi dikirim modul WIT, bukan tafsiran lain.
    """
    roll = math.atan2(
        2.0 * (w * x + y * z),
        1.0 - 2.0 * (x * x + y * y),
    )

    sinp = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(sinp)

    yaw = math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )

    return roll, pitch, yaw


class Mapping3DImu(Node):

    def __init__(self):
        super().__init__('mapping_3d_imu')

        self.declare_parameter('frame_id', 'base_link')
        # True = rotasi -sudut (sama seperti mapping_3d_sweep.py).
        self.declare_parameter('invert_rotation', True)
        self.declare_parameter('publish_every_sweep', True)
        self.declare_parameter('final_publish_delay', 0.5)
        self.declare_parameter('max_points', 0)
        self.declare_parameter('health_check_period', 5.0)
        # Koreksi kemiringan tetap terhadap sumbu putar stepper, dalam derajat.
        self.declare_parameter('tilt_offset_deg', 0.0)

        self.declare_parameter('imu_topic', '/imu/data_raw')
        # Koreksi yaw. Default mati: yaw dari magnetometer tidak bisa dipercaya
        # di dekat motor drone, dan yaw palsu merusak peta yang tadinya bagus.
        self.declare_parameter('use_yaw', False)
        # Bias pemasangan IMU terhadap bidang LiDAR. Cara mengisi: taruh rig di
        # meja yang datar, jalankan node ini, baca angka roll/pitch yang
        # dilaporkan saat pesan IMU pertama masuk, lalu masukkan angka itu
        # apa adanya di sini.
        self.declare_parameter('imu_roll_offset_deg', 0.0)
        self.declare_parameter('imu_pitch_offset_deg', 0.0)

        self.frame_id = str(self.get_parameter('frame_id').value)
        self.rotation_sign = -1.0 if self.get_parameter('invert_rotation').value else 1.0
        self.publish_every_sweep = bool(self.get_parameter('publish_every_sweep').value)
        self.final_publish_delay = float(self.get_parameter('final_publish_delay').value)
        self.max_points = int(self.get_parameter('max_points').value)
        self.tilt_offset = math.radians(
            float(self.get_parameter('tilt_offset_deg').value))

        self.use_yaw = bool(self.get_parameter('use_yaw').value)
        self.roll_offset = math.radians(
            float(self.get_parameter('imu_roll_offset_deg').value))
        self.pitch_offset = math.radians(
            float(self.get_parameter('imu_pitch_offset_deg').value))

        self.servo_times = []
        self.servo_angles = []

        self.imu_times = []
        self.imu_rolls = []
        self.imu_pitches = []
        self.imu_yaws = []
        # Yaw dijaga kontinu (tanpa lompatan +-pi) supaya np.interp tidak
        # menghasilkan putaran palsu saat melewati batas wrap.
        self.yaw_continuous = 0.0
        self.last_raw_yaw = None
        self.yaw_reference = None

        self.chunks = []
        self.total_points = 0
        # Titik yang masuk tanpa koreksi IMU karena orientasi belum tersedia.
        # Dihitung supaya kalau IMU mati, hasilnya tidak diam-diam dianggap
        # terkoreksi.
        self.uncorrected_points = 0
        self.sweeps_seen = 0
        self.done = False
        self.final_timer = None

        self.scans_received = 0
        self.imu_received = 0
        self.health_ticks = 0

        imu_topic = str(self.get_parameter('imu_topic').value)

        self.create_subscription(Float32, '/stepper/angle', self.servo_callback, 50)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(Imu, imu_topic, self.imu_callback, 50)
        self.create_subscription(
            Int32, '/stepper/sweep_count', self.sweep_count_callback, latched_qos()
        )
        self.create_subscription(
            Bool, '/stepper/sweep_done', self.sweep_done_callback, latched_qos()
        )

        self.map_pub = self.create_publisher(PointCloud2, '/map_3d', 10)

        period = float(self.get_parameter('health_check_period').value)
        if period > 0:
            self.create_timer(period, self.health_check)

        self.get_logger().info(
            f'Mapping3DImu started (frame={self.frame_id}, '
            f'rotation_sign={self.rotation_sign:+.0f}, '
            f'tilt_offset={math.degrees(self.tilt_offset):+.2f} deg, '
            f'imu={imu_topic}, use_yaw={self.use_yaw}). '
            'Menunggu data sweep...'
        )

    # ==========================================================
    # Utility
    # ==========================================================

    @staticmethod
    def header_time_sec(header):
        return header.stamp.sec + header.stamp.nanosec * 1e-9

    def now_sec(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def has_imu(self):
        return len(self.imu_times) >= 2

    # ==========================================================
    # Pemantauan kesehatan masukan
    # ==========================================================

    def health_check(self):
        """Teriak lebih awal kalau masukan tidak sehat.

        Tanpa ini, node terlihat baik-baik saja lalu baru mengaku '0 titik'
        setelah seluruh sweep selesai (bisa semenit lebih terbuang).
        """
        if self.done:
            return

        self.health_ticks += 1
        has_servo = len(self.servo_times) >= 2

        if self.scans_received == 0 and not has_servo:
            self.get_logger().error(
                'Tidak ada /scan DAN tidak ada /stepper/angle. '
                'Cek: ros2 topic hz /scan'
            )
        elif self.scans_received == 0:
            self.get_logger().error(
                'Stepper jalan tapi /scan KOSONG - LiDAR tidak mengirim data. '
                'Cek: ros2 topic hz /scan'
            )
        elif not has_servo:
            self.get_logger().error(
                f'/scan masuk ({self.scans_received} pesan) tapi /stepper/angle '
                'kosong. Titik tidak bisa dirotasi tanpa sudut stepper.'
            )
        elif not self.has_imu():
            self.get_logger().error(
                'IMU KOSONG - titik tetap dikumpulkan tapi TANPA koreksi '
                'orientasi. Hasilnya setara sweep_mapping biasa, jangan '
                'diklaim terkoreksi. Cek: ros2 topic hz /imu/data_raw'
            )
        elif self.total_points == 0:
            self.get_logger().error(
                f'/scan masuk {self.scans_received} pesan tapi 0 titik lolos '
                'filter jarak. Semua pembacaan di luar range_min/range_max?'
            )
        elif self.health_ticks % 3 == 0:
            self.get_logger().info(
                f'Sehat: {self.scans_received} scan, {self.imu_received} IMU, '
                f'{self.total_points} titik'
            )

    # ==========================================================
    # Callback masukan
    # ==========================================================

    def servo_callback(self, msg):
        self.servo_times.append(self.now_sec())
        self.servo_angles.append(float(msg.data))

        if len(self.servo_times) > SERVO_BUFFER:
            self.servo_times.pop(0)
            self.servo_angles.pop(0)

    def imu_callback(self, msg):
        q = msg.orientation
        roll, pitch, yaw = quat_to_rpy(q.x, q.y, q.z, q.w)

        if self.last_raw_yaw is None:
            self.yaw_continuous = yaw
            self.yaw_reference = yaw
            self.get_logger().info(
                'IMU pertama masuk - roll='
                f'{math.degrees(roll):+.2f} deg, pitch={math.degrees(pitch):+.2f} deg. '
                'Kalau rig sedang datar, dua angka ini adalah bias pemasangan: '
                'salin ke imu_roll_offset_deg / imu_pitch_offset_deg.'
            )
        else:
            # Jaga yaw tetap kontinu dengan menjumlahkan selisih yang sudah
            # dibungkus ke [-pi, pi].
            delta = yaw - self.last_raw_yaw
            delta = (delta + math.pi) % (2.0 * math.pi) - math.pi
            self.yaw_continuous += delta

        self.last_raw_yaw = yaw
        self.imu_received += 1

        self.imu_times.append(self.now_sec())
        self.imu_rolls.append(roll - self.roll_offset)
        self.imu_pitches.append(pitch - self.pitch_offset)
        # Yaw dipakai relatif terhadap awal sweep, bukan terhadap utara magnetik.
        self.imu_yaws.append(self.yaw_continuous - self.yaw_reference)

        if len(self.imu_times) > IMU_BUFFER:
            self.imu_times.pop(0)
            self.imu_rolls.pop(0)
            self.imu_pitches.pop(0)
            self.imu_yaws.pop(0)

    def sweep_count_callback(self, msg):
        count = int(msg.data)
        if count <= self.sweeps_seen:
            return

        self.sweeps_seen = count
        self.get_logger().info(
            f'Sweep {count} terkumpul - total {self.total_points} titik'
        )

        if self.publish_every_sweep and not self.done:
            self.publish_cloud()

    def sweep_done_callback(self, msg):
        if not msg.data or self.done:
            return

        self.done = True
        self.get_logger().info(
            f'Sweep selesai semua. Menunggu {self.final_publish_delay:.1f} detik '
            'untuk scan terakhir...'
        )
        self.final_timer = self.create_timer(
            self.final_publish_delay, self.publish_final
        )

    def publish_final(self):
        if self.final_timer is not None:
            self.final_timer.cancel()
            self.destroy_timer(self.final_timer)
            self.final_timer = None

        self.publish_cloud()
        self.get_logger().info(
            f'=== SCAN SELESAI === {self.sweeps_seen} sweep, '
            f'{self.total_points} titik di topic /map_3d (frame {self.frame_id}). '
            'Cloud tetap di-latch di memori; Ctrl+C untuk keluar.'
        )

    # ==========================================================
    # LiDAR callback
    # ==========================================================

    def apply_imu_rotation(self, vx, vy, vz, ray_times):
        """Putar titik dengan orientasi rangka dari IMU.

        Susunannya R_z(yaw) * R_y(pitch) * R_x(roll), sama dengan konvensi ZYX
        yang dipakai quat_to_rpy. Dikerjakan elemen per elemen dengan numpy,
        bukan lewat N buah matriks 3x3, supaya tidak ada alokasi besar per scan.
        """
        times = np.asarray(self.imu_times)

        roll = np.interp(ray_times, times, np.asarray(self.imu_rolls))
        pitch = np.interp(ray_times, times, np.asarray(self.imu_pitches))

        cr, sr = np.cos(roll), np.sin(roll)
        ux = vx
        uy = vy * cr - vz * sr
        uz = vy * sr + vz * cr

        cp, sp = np.cos(pitch), np.sin(pitch)
        wx = ux * cp + uz * sp
        wy = uy
        wz = -ux * sp + uz * cp

        if not self.use_yaw:
            return wx, wy, wz

        yaw = np.interp(ray_times, times, np.asarray(self.imu_yaws))
        cy, sy = np.cos(yaw), np.sin(yaw)
        return wx * cy - wy * sy, wx * sy + wy * cy, wz

    def scan_callback(self, msg):
        # Dihitung sebelum penjaga apa pun, supaya health_check bisa membedakan
        # "topic tidak ada" dari "topic ada tapi datanya ditolak".
        self.scans_received += 1

        if self.done or len(self.servo_times) < 2:
            return

        if self.max_points > 0 and self.total_points >= self.max_points:
            return

        ranges = np.asarray(msg.ranges, dtype=np.float64)
        if ranges.size == 0:
            return

        # Buang dulu sinar yang tidak valid, baru hitung geometri. Selain lebih cepat,
        # ini mencegah inf/nan ikut dihitung dan memicu RuntimeWarning.
        with np.errstate(invalid='ignore'):
            valid = (
                (ranges > msg.range_min)
                & (ranges < msg.range_max)
                & np.isfinite(ranges)
            )

        keep = np.flatnonzero(valid)
        if self.max_points > 0:
            keep = keep[:self.max_points - self.total_points]

        count = keep.size
        if count == 0:
            return

        time_increment = msg.time_increment
        if time_increment <= 0.0:
            time_increment = (1.0 / 10.0) / ranges.size

        scan_start = self.header_time_sec(msg.header)
        offset_times = keep * time_increment
        ray_times = scan_start + offset_times

        # Sudut stepper untuk tiap sinar, diinterpolasi dari riwayat /stepper/angle.
        servo_angle = np.interp(
            ray_times,
            np.asarray(self.servo_times),
            np.asarray(self.servo_angles),
        )

        r = ranges[keep]
        scan_angle = msg.angle_min + keep * msg.angle_increment

        # Titik di frame LiDAR (bidang datar, z = 0).
        x = r * np.cos(scan_angle)
        y = r * np.sin(scan_angle)

        # Rotasi stepper terhadap sumbu X. Karena z = 0, hasilnya murah:
        # x' = x, y' = y*cos(a), z' = y*sin(a)
        a = self.rotation_sign * servo_angle + self.tilt_offset
        vx = x
        vy = y * np.cos(a)
        vz = y * np.sin(a)

        # Rotasi kedua: orientasi rangka dari IMU. Kalau IMU belum ada, titik
        # tetap dimasukkan apa adanya dan dihitung sebagai tak terkoreksi -
        # membuang data penerbangan lebih mahal daripada melaporkannya apa adanya.
        if self.has_imu():
            px, py, pz = self.apply_imu_rotation(vx, vy, vz, ray_times)
        else:
            px, py, pz = vx, vy, vz
            self.uncorrected_points += count

        intensities = np.asarray(msg.intensities, dtype=np.float32)
        if intensities.size < ranges.size:
            padded = np.zeros(ranges.size, dtype=np.float32)
            padded[:intensities.size] = intensities
            intensities = padded

        chunk = np.empty(count, dtype=POINT_DTYPE)
        chunk['x'] = px
        chunk['y'] = py
        chunk['z'] = pz
        chunk['intensity'] = intensities[keep]
        chunk['ring'] = 0
        chunk['time'] = offset_times

        self.chunks.append(chunk)
        self.total_points += count

    # ==========================================================
    # Publish
    # ==========================================================

    def publish_cloud(self):
        if self.total_points < 100:
            self.get_logger().warn(
                f'Cuma {self.total_points} titik, cloud tidak di-publish. '
                'Cek apakah /scan dan /stepper/angle jalan.'
            )
            return

        self.chunks = merge_chunks(self.chunks, self.total_points)
        points = self.chunks[0]

        cloud = make_cloud(
            points, self.frame_id, self.get_clock().now().to_msg()
        )
        self.map_pub.publish(cloud)

        note = ''
        if self.uncorrected_points:
            note = (
                f', {self.uncorrected_points} titik TANPA koreksi IMU'
            )

        self.get_logger().info(
            f'Published cloud: {cloud.width} titik '
            f'({cloud.row_step / 1e6:.1f} MB), sweep {self.sweeps_seen}{note}'
        )

    def destroy_node(self):
        self.get_logger().info(
            f'Mapping3DImu stopped - {self.total_points} titik terkumpul, '
            f'{self.uncorrected_points} tanpa koreksi IMU'
        )
        super().destroy_node()


def main():
    rclpy.init()
    node = Mapping3DImu()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
