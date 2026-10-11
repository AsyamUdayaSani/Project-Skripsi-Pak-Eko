"""Mapping 3D dengan koreksi IMU, untuk rig yang dipasang di drone.

PERBAIKAN dari versi sebelumnya:
  * Rotasi stepper: sumbu Y (bukan X). LiDAR di-tilt ke depan/belakang via
    belt horizontal, sehingga sumbu putarnya adalah Y (pitch axis), bukan X (roll axis).
  * apply_imu_rotation sekarang memakai urutan R_x(roll) → R_y(pitch) → R_z(yaw)
    yang konsisten dengan konvensi ZYX Tait-Bryan dari quat_to_rpy.
  * Ditambah parameter `stepper_axis` ('x'/'y'/'z') untuk eksperimen tanpa edit kode.

Urutan dua rotasi (tidak berubah - ini sudah benar di versi lama):
  1. Rotasi stepper  : titik di frame LiDAR → frame sensor-head (mengikuti sudut tilt)
  2. Rotasi IMU      : frame sensor-head → frame dunia (kompensasi goyangan drone)
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

IMU_BUFFER = 400
SERVO_BUFFER = 100


def quat_to_rpy(x, y, z, w):
    """Quaternion -> roll, pitch, yaw (radian), konvensi ZYX Tait-Bryan."""
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
        self.declare_parameter('invert_rotation', True)
        self.declare_parameter('publish_every_sweep', True)
        self.declare_parameter('final_publish_delay', 0.5)
        self.declare_parameter('max_points', 0)
        self.declare_parameter('health_check_period', 5.0)
        self.declare_parameter('tilt_offset_deg', 0.0)

        self.declare_parameter('imu_topic', '/imu/data_raw')
        self.declare_parameter('use_yaw', False)
        self.declare_parameter('imu_roll_offset_deg', 0.0)
        self.declare_parameter('imu_pitch_offset_deg', 0.0)

        # PERBAIKAN: parameter sumbu stepper. Default 'y' karena belt horizontal
        # memutar LiDAR sekitar sumbu Y (maju-mundur / pitch axis).
        # Ganti ke 'x' kalau rig diputar ke kiri-kanan / roll axis.
        self.declare_parameter('stepper_axis', 'y')

        self.frame_id = str(self.get_parameter('frame_id').value)
        self.rotation_sign = -1.0 if self.get_parameter('invert_rotation').value else 1.0
        self.publish_every_sweep = bool(self.get_parameter('publish_every_sweep').value)
        self.final_publish_delay = float(self.get_parameter('final_publish_delay').value)
        self.max_points = int(self.get_parameter('max_points').value)
        self.tilt_offset = math.radians(float(self.get_parameter('tilt_offset_deg').value))
        self.use_yaw = bool(self.get_parameter('use_yaw').value)
        self.roll_offset = math.radians(float(self.get_parameter('imu_roll_offset_deg').value))
        self.pitch_offset = math.radians(float(self.get_parameter('imu_pitch_offset_deg').value))
        self.stepper_axis = str(self.get_parameter('stepper_axis').value).lower()

        if self.stepper_axis not in ('x', 'y', 'z'):
            self.get_logger().warn(
                f"stepper_axis='{self.stepper_axis}' tidak valid, pakai 'y'"
            )
            self.stepper_axis = 'y'

        self.servo_times = []
        self.servo_angles = []

        self.imu_times = []
        self.imu_rolls = []
        self.imu_pitches = []
        self.imu_yaws = []
        self.yaw_continuous = 0.0
        self.last_raw_yaw = None
        self.yaw_reference = None

        self.chunks = []
        self.total_points = 0
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
            f'stepper_axis={self.stepper_axis}, '
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
    # Health check
    # ==========================================================

    def health_check(self):
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
                'orientasi. Cek: ros2 topic hz /imu/data_raw'
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
    # Callbacks
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
                f'IMU pertama masuk - roll={math.degrees(roll):+.2f} deg, '
                f'pitch={math.degrees(pitch):+.2f} deg. '
                'Kalau rig sedang datar, salin ke imu_roll_offset_deg / imu_pitch_offset_deg.'
            )
        else:
            delta = yaw - self.last_raw_yaw
            delta = (delta + math.pi) % (2.0 * math.pi) - math.pi
            self.yaw_continuous += delta

        self.last_raw_yaw = yaw
        self.imu_received += 1

        self.imu_times.append(self.now_sec())
        self.imu_rolls.append(roll - self.roll_offset)
        self.imu_pitches.append(pitch - self.pitch_offset)
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
        self.get_logger().info(f'Sweep {count} terkumpul - total {self.total_points} titik')
        if self.publish_every_sweep and not self.done:
            self.publish_cloud()

    def sweep_done_callback(self, msg):
        if not msg.data or self.done:
            return
        self.done = True
        self.get_logger().info(
            f'Sweep selesai semua. Menunggu {self.final_publish_delay:.1f} detik...'
        )
        self.final_timer = self.create_timer(self.final_publish_delay, self.publish_final)

    def publish_final(self):
        if self.final_timer is not None:
            self.final_timer.cancel()
            self.destroy_timer(self.final_timer)
            self.final_timer = None
        self.publish_cloud()
        self.get_logger().info(
            f'=== SCAN SELESAI === {self.sweeps_seen} sweep, '
            f'{self.total_points} titik di /map_3d (frame {self.frame_id}). '
            'Ctrl+C untuk keluar.'
        )

    # ==========================================================
    # PERBAIKAN UTAMA: Rotasi stepper & IMU
    # ==========================================================

    def _rotate_stepper(self, x, y, z, angle):
        """Rotasi sumbu stepper ke titik 3D (vectorized numpy).

        Geometri alat (dari sketsa & foto):
          - Poros pipa = horizontal melintang = SUMBU Y (pitch axis)
          - LiDAR terpasang di ujung lengan, menghadap ke depan
          - Bidang scan LiDAR awalnya di bidang XZ (sebelum rotasi)

        Karena z_local = 0 (titik 2D dari LiDAR), rotasi sumbu Y menjadi:
          vx =  x * cos(a)      ← komponen maju-mundur berputar
          vy =  y               ← sumbu Y (melintang) tidak berubah
          vz = -x * sin(a)      ← komponen vertikal muncul dari x

        Ini menghasilkan 'baling-baling vertikal': saat a=0 titik ada di
        bidang horizontal, saat a=90° titik ada di atas/bawah.

        Sumbu lain tersedia lewat parameter stepper_axis untuk eksperimen.
        """
        c, s = np.cos(angle), np.sin(angle)

        if self.stepper_axis == 'y':
            # Rotasi sumbu Y — BENAR untuk poros pipa horizontal melintang
            # Input z=0 (LiDAR 2D), jadi z*cos/sin tidak berkontribusi
            rx = x * c + z * s   # z=0 → rx = x*cos(a)
            ry = y                # sumbu rotasi tidak berubah
            rz = -x * s + z * c  # z=0 → rz = -x*sin(a)
        elif self.stepper_axis == 'x':
            # Rotasi sumbu X — untuk poros pipa horizontal memanjang
            rx = x
            ry = y * c - z * s
            rz = y * s + z * c
        else:  # 'z'
            # Rotasi sumbu Z — untuk poros vertikal (seperti radar horizontal)
            rx = x * c - y * s
            ry = x * s + y * c
            rz = z

        return rx, ry, rz

    def _apply_imu_rotation(self, vx, vy, vz, ray_times):
        """Rotasi IMU: R_x(roll) → R_y(pitch) → R_z(yaw), ZYX Tait-Bryan."""
        times = np.asarray(self.imu_times)

        # Step 1 – Roll (sumbu X)
        roll = np.interp(ray_times, times, np.asarray(self.imu_rolls))
        cr, sr = np.cos(roll), np.sin(roll)
        ux = vx
        uy = vy * cr - vz * sr
        uz = vy * sr + vz * cr

        # Step 2 – Pitch (sumbu Y)
        pitch = np.interp(ray_times, times, np.asarray(self.imu_pitches))
        cp, sp = np.cos(pitch), np.sin(pitch)
        wx = ux * cp + uz * sp
        wy = uy
        wz = -ux * sp + uz * cp

        if not self.use_yaw:
            return wx, wy, wz

        # Step 3 – Yaw (sumbu Z), opsional
        yaw = np.interp(ray_times, times, np.asarray(self.imu_yaws))
        cy, sy = np.cos(yaw), np.sin(yaw)
        return wx * cy - wy * sy, wx * sy + wy * cy, wz

    def scan_callback(self, msg):
        self.scans_received += 1

        if self.done or len(self.servo_times) < 2:
            return
        if self.max_points > 0 and self.total_points >= self.max_points:
            return

        ranges = np.asarray(msg.ranges, dtype=np.float64)
        if ranges.size == 0:
            return

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

        # Sudut stepper per sinar
        servo_angle = np.interp(
            ray_times,
            np.asarray(self.servo_times),
            np.asarray(self.servo_angles),
        )

        r = ranges[keep]
        scan_angle = msg.angle_min + keep * msg.angle_increment

        # Titik di frame LiDAR (bidang 2D, z = 0)
        x_local = r * np.cos(scan_angle)
        y_local = r * np.sin(scan_angle)
        z_local = np.zeros_like(x_local)

        # === ROTASI 1: Stepper (tilt LiDAR) ===
        # Mengubah titik dari frame LiDAR 2D → frame sensor-head 3D
        a = self.rotation_sign * servo_angle + self.tilt_offset
        vx, vy, vz = self._rotate_stepper(x_local, y_local, z_local, a)

        # === ROTASI 2: IMU (orientasi drone) ===
        # Mengubah titik dari frame sensor-head → frame dunia
        if self.has_imu():
            px, py, pz = self._apply_imu_rotation(vx, vy, vz, ray_times)
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

        cloud = make_cloud(points, self.frame_id, self.get_clock().now().to_msg())
        self.map_pub.publish(cloud)

        note = ''
        if self.uncorrected_points:
            note = f', {self.uncorrected_points} titik TANPA koreksi IMU'

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
