"""Mapping 3D yang mengakumulasi N sweep menjadi satu pointcloud utuh.

Perbedaan dengan stepper_controller/mapping_3d.py:
  * Buffer titik TIDAK dikosongkan tiap sweep -> semua sweep menumpuk jadi satu cloud.
  * Batas sweep dibaca dari /stepper/sweep_count (dari stepper_sweep_node),
    bukan ditebak dari wrap-around sudut.
  * frame_id = base_link karena titik sudah di-de-rotasi ke frame diam.
    (mapping_3d.py memakai lidar_tilt dan hanya benar karena publish-nya
    kebetulan selalu pas sudut ~0. Di sini cloud di-publish di sudut sembarang.)
  * Perhitungan titik divektorisasi dengan numpy, bukan loop Python per titik.

File asli tidak diubah sama sekali.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_msgs.msg import Bool, Float32, Header, Int32

# Tata letak satu titik, sama persis dengan struct.pack('ffffH2xf') = 24 byte.
POINT_DTYPE = np.dtype({
    'names': ['x', 'y', 'z', 'intensity', 'ring', 'time'],
    'formats': ['<f4', '<f4', '<f4', '<f4', '<u2', '<f4'],
    'offsets': [0, 4, 8, 12, 16, 20],
    'itemsize': 24,
})

POINT_FIELDS = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
    PointField(name='ring', offset=16, datatype=PointField.UINT16, count=1),
    PointField(name='time', offset=20, datatype=PointField.FLOAT32, count=1),
]


def latched_qos():
    return QoSProfile(
        depth=1,
        history=QoSHistoryPolicy.KEEP_LAST,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    )


class Mapping3DSweep(Node):

    def __init__(self):
        super().__init__('mapping_3d_sweep')

        self.declare_parameter('frame_id', 'base_link')
        # True = rotasi -sudut (sama seperti mapping_3d.py yang dipakai sekarang).
        # Ubah ke False kalau hasil scan ternyata terbalik/kecermin.
        self.declare_parameter('invert_rotation', True)
        # Publish cloud kumulatif setiap satu sweep selesai (progres kelihatan di viewer).
        self.declare_parameter('publish_every_sweep', True)
        # Tunggu sebentar setelah sweep terakhir supaya scan yang masih di jalan ikut masuk.
        self.declare_parameter('final_publish_delay', 0.5)
        # Batas jumlah titik demi keamanan RAM. 0 = tanpa batas.
        self.declare_parameter('max_points', 0)

        self.frame_id = str(self.get_parameter('frame_id').value)
        self.rotation_sign = -1.0 if self.get_parameter('invert_rotation').value else 1.0
        self.publish_every_sweep = bool(self.get_parameter('publish_every_sweep').value)
        self.final_publish_delay = float(self.get_parameter('final_publish_delay').value)
        self.max_points = int(self.get_parameter('max_points').value)

        self.servo_times = []
        self.servo_angles = []
        self.chunks = []
        self.total_points = 0
        self.sweeps_seen = 0
        self.done = False
        self.final_timer = None

        self.create_subscription(Float32, '/stepper/angle', self.servo_callback, 50)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(
            Int32, '/stepper/sweep_count', self.sweep_count_callback, latched_qos()
        )
        self.create_subscription(
            Bool, '/stepper/sweep_done', self.sweep_done_callback, latched_qos()
        )

        self.map_pub = self.create_publisher(PointCloud2, '/map_3d', 10)

        self.get_logger().info(
            f'Mapping3DSweep started (frame={self.frame_id}, '
            f'rotation_sign={self.rotation_sign:+.0f}). '
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

    # ==========================================================
    # Stepper callbacks
    # ==========================================================

    def servo_callback(self, msg):
        self.servo_times.append(self.now_sec())
        self.servo_angles.append(float(msg.data))

        if len(self.servo_times) > 100:
            self.servo_times.pop(0)
            self.servo_angles.pop(0)

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

    def scan_callback(self, msg):
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

        # Rotasi terhadap sumbu X sebesar sudut stepper. Karena z=0, hasilnya:
        # x' = x, y' = y*cos(a), z' = y*sin(a)
        a = self.rotation_sign * servo_angle

        intensities = np.asarray(msg.intensities, dtype=np.float32)
        if intensities.size < ranges.size:
            padded = np.zeros(ranges.size, dtype=np.float32)
            padded[:intensities.size] = intensities
            intensities = padded

        chunk = np.empty(count, dtype=POINT_DTYPE)
        chunk['x'] = x
        chunk['y'] = y * np.cos(a)
        chunk['z'] = y * np.sin(a)
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

        # Gabungkan semua chunk jadi satu blok kontinu supaya tidak digabung berulang.
        # CATATAN: jangan pakai np.concatenate di sini. Untuk dtype bercelah (padding
        # 2 byte setelah 'ring'), concatenate memampatkan itemsize 24 -> 22 sehingga
        # data tidak lagi cocok dengan point_step dan cloud jadi kacau di viewer.
        if len(self.chunks) > 1:
            merged = np.empty(self.total_points, dtype=POINT_DTYPE)
            offset = 0
            for chunk in self.chunks:
                merged[offset:offset + chunk.size] = chunk
                offset += chunk.size
            self.chunks = [merged]
        points = self.chunks[0]

        assert points.dtype.itemsize == 24, 'padding titik hilang'

        cloud = PointCloud2()
        cloud.header = Header()
        cloud.header.stamp = self.get_clock().now().to_msg()
        cloud.header.frame_id = self.frame_id
        cloud.height = 1
        cloud.width = points.size
        cloud.fields = POINT_FIELDS
        cloud.is_bigendian = False
        cloud.point_step = POINT_DTYPE.itemsize
        cloud.row_step = cloud.point_step * cloud.width
        cloud.data = points.tobytes()
        cloud.is_dense = True

        self.map_pub.publish(cloud)

        self.get_logger().info(
            f'Published cloud: {cloud.width} titik '
            f'({cloud.row_step / 1e6:.1f} MB), sweep {self.sweeps_seen}'
        )

    def destroy_node(self):
        self.get_logger().info(
            f'Mapping3DSweep stopped - {self.total_points} titik terkumpul'
        )
        super().destroy_node()


def main():
    rclpy.init()
    node = Mapping3DSweep()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
