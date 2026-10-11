"""Mapping 3D yang hanya memetakan fase 180 derajat dari tiap sweep.

Pasangan stepper_sweep_node.py: satu sweep = 180 derajat dipetakan + 180 derajat
pulang cepat. Scan yang jatuh di fase pulang DIBUANG, supaya ruang yang sama tidak
terpetakan dua kali (penyebab objek dobel di versi lama, mapping_3d_sweep1.py).

Fase dibaca dari /stepper/mapping_active. Setiap perubahan dicatat dengan waktu
terimanya - jam yang sama dengan riwayat /stepper/angle - lalu tiap SINAR (bukan
tiap pesan scan) diperiksa apakah waktunya jatuh di dalam jendela mapping. Satu
pesan /scan yang kebetulan melintasi batas 180 derajat terpotong tepat di batas.

Kalau /stepper/mapping_active tidak pernah datang (misal memutar ulang bag lama
dari stepper_sweep_node1), semua scan dipakai seperti versi lama dan health_check
memberi peringatan.

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

import math

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import Bool, Float32, Int32

from sweep_mapping.cloud_utils import (
    POINT_DTYPE,
    latched_qos,
    make_cloud,
    merge_chunks,
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
        # Selang pengecekan kesehatan masukan. 0 = matikan.
        self.declare_parameter('health_check_period', 5.0)
        # Koreksi kemiringan tetap, dalam derajat, terhadap sumbu putar stepper.
        # Dipakai kalau posisi awal LiDAR ternyata tidak benar-benar datar:
        # ukur kemiringan peta di CloudCompare, masukkan di sini dengan tanda
        # berlawanan. Menggeser sudut seluruh titik, setara memutar seluruh
        # cloud sebesar nilai ini terhadap sumbu X.
        self.declare_parameter('tilt_offset_deg', 0.0)

        self.frame_id = str(self.get_parameter('frame_id').value)
        self.rotation_sign = -1.0 if self.get_parameter('invert_rotation').value else 1.0
        self.publish_every_sweep = bool(self.get_parameter('publish_every_sweep').value)
        self.final_publish_delay = float(self.get_parameter('final_publish_delay').value)
        self.max_points = int(self.get_parameter('max_points').value)
        self.tilt_offset = math.radians(
            float(self.get_parameter('tilt_offset_deg').value))

        self.servo_times = []
        self.servo_angles = []
        # Jendela waktu fase MAPPING: [mulai, selesai]; selesai=None = masih jalan.
        self.mapping_windows = []
        self.mapping_gate_seen = False
        self.gate_warned = False
        self.chunks = []
        self.total_points = 0
        self.sweeps_seen = 0
        self.done = False
        self.final_timer = None

        self.scans_received = 0
        self.health_ticks = 0

        self.create_subscription(Float32, '/stepper/angle', self.servo_callback, 50)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(
            Int32, '/stepper/sweep_count', self.sweep_count_callback, latched_qos()
        )
        self.create_subscription(
            Bool, '/stepper/sweep_done', self.sweep_done_callback, latched_qos()
        )
        self.create_subscription(
            Bool, '/stepper/mapping_active', self.mapping_active_callback,
            latched_qos()
        )

        # Latched: cloud terakhir disimpan, jadi Foxglove/RViz yang baru tersambung
        # SETELAH cloud dikirim (misal scan sudah selesai) tetap langsung menerimanya.
        self.map_pub = self.create_publisher(PointCloud2, '/map_3d', latched_qos())

        period = float(self.get_parameter('health_check_period').value)
        if period > 0:
            self.create_timer(period, self.health_check)

        self.get_logger().info(
            f'Mapping3DSweep started (frame={self.frame_id}, '
            f'rotation_sign={self.rotation_sign:+.0f}, '
            f'tilt_offset={math.degrees(self.tilt_offset):+.2f} deg). '
            'Hanya fase mapping 180 derajat yang dipetakan. Menunggu data sweep...'
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

        if (not self.mapping_gate_seen and not self.gate_warned
                and self.scans_received > 0 and has_servo):
            self.gate_warned = True
            self.get_logger().warn(
                '/stepper/mapping_active belum pernah datang - SEMUA scan dipakai, '
                'termasuk fase pulang (hasil bisa dobel seperti versi 360 derajat). '
                'Pastikan yang jalan stepper_sweep_node, bukan stepper_sweep_node1.'
            )

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
        elif self.total_points == 0:
            self.get_logger().error(
                f'/scan masuk {self.scans_received} pesan tapi 0 titik lolos '
                'filter jarak. Semua pembacaan di luar range_min/range_max?'
            )
        elif self.health_ticks % 3 == 0:
            self.get_logger().info(
                f'Sehat: {self.scans_received} scan, {self.total_points} titik'
            )

    # ==========================================================
    # Stepper callbacks
    # ==========================================================

    def servo_callback(self, msg):
        self.servo_times.append(self.now_sec())
        self.servo_angles.append(float(msg.data))

        if len(self.servo_times) > 100:
            self.servo_times.pop(0)
            self.servo_angles.pop(0)

    def mapping_active_callback(self, msg):
        t = self.now_sec()
        self.mapping_gate_seen = True
        windows = self.mapping_windows

        if msg.data:
            if not windows or windows[-1][1] is not None:
                windows.append([t, None])
        elif windows and windows[-1][1] is None:
            windows[-1][1] = t

    def in_mapping_window(self, times):
        """Mask bool: sinar mana yang waktunya jatuh di fase MAPPING."""
        mask = np.zeros(times.shape, dtype=bool)
        for start, end in self.mapping_windows:
            inside = times >= start
            if end is not None:
                inside &= times <= end
            mask |= inside
        return mask

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

        time_increment = msg.time_increment
        if time_increment <= 0.0:
            time_increment = (1.0 / 10.0) / ranges.size

        scan_start = self.header_time_sec(msg.header)
        ray_times = scan_start + keep * time_increment

        # Buang sinar yang diambil di fase pulang (atau sebelum mapping mulai).
        if self.mapping_gate_seen:
            inside = self.in_mapping_window(ray_times)
            keep = keep[inside]
            ray_times = ray_times[inside]

        if self.max_points > 0:
            sisa = self.max_points - self.total_points
            keep = keep[:sisa]
            ray_times = ray_times[:sisa]

        count = keep.size
        if count == 0:
            return

        offset_times = keep * time_increment

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
        # tilt_offset ditambahkan sebagai konstanta: karena semua rotasi di sini
        # sekitar sumbu yang sama, menambah konstanta ke sudut sama persis dengan
        # memutar seluruh cloud jadi sekali di akhir.
        a = self.rotation_sign * servo_angle + self.tilt_offset

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
        self.chunks = merge_chunks(self.chunks, self.total_points)
        points = self.chunks[0]

        cloud = make_cloud(
            points, self.frame_id, self.get_clock().now().to_msg()
        )
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
