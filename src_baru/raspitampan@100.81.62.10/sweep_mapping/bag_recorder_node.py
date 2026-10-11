"""Auto-record: `ros2 bag record` jalan sendiri, berhenti sendiri saat sweep selesai.

Alur:
  1. Node nyala -> cari index kosong berikutnya di folder bag (0001, 0002, ...).
  2. Langsung jalankan `ros2 bag record` sebagai subprocess.
  3. Menunggu /stepper/sweep_done dari stepper_sweep_node.
  4. Tunggu `stop_delay` detik (biar cloud final dari mapping ikut terekam),
     lalu kirim SIGINT ke recorder supaya metadata.yaml ditutup rapi.

Kalau num_sweeps=0 (muter terus), rekaman berjalan sampai Ctrl+C dan tetap
ditutup dengan rapi.
"""

import os
import re
import shutil
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from std_msgs.msg import Bool

DEFAULT_TOPICS = [
    '/scan',
    '/imu/data_raw',
    '/stepper/angle',
    '/stepper/steps',
    '/stepper/status',
    '/stepper/sweep_count',
    '/stepper/sweep_done',
    '/map_3d',
    '/odom',
    '/tf',
    '/tf_static',
]


def latched_qos():
    return QoSProfile(
        depth=1,
        history=QoSHistoryPolicy.KEEP_LAST,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    )


def dir_size_mb(path):
    total = 0
    for root, _dirs, files in os.walk(path):
        for f in files:
            try:
                total += os.path.getsize(os.path.join(root, f))
            except OSError:
                pass
    return total / 1e6


class BagRecorderNode(Node):

    def __init__(self):
        super().__init__('bag_recorder')

        self.declare_parameter('bag_dir', '~/bags')
        self.declare_parameter('bag_prefix', 'scan')
        # Dipakai untuk penamaan saja, biar gampang membandingkan hasil antar jumlah sweep.
        self.declare_parameter('num_sweeps', 1)
        self.declare_parameter('topics', DEFAULT_TOPICS)
        self.declare_parameter('storage', 'mcap')
        self.declare_parameter('compress', False)
        # Jeda setelah sweep_done sebelum recorder dimatikan, supaya cloud final
        # dari mapping_3d_sweep (yang di-publish 0.5 detik setelah sweep_done) ikut terekam.
        self.declare_parameter('stop_delay', 3.0)

        self.bag_dir = os.path.expanduser(str(self.get_parameter('bag_dir').value))
        self.bag_prefix = str(self.get_parameter('bag_prefix').value)
        self.num_sweeps = int(self.get_parameter('num_sweeps').value)
        self.topics = list(self.get_parameter('topics').value)
        self.storage = str(self.get_parameter('storage').value)
        self.compress = bool(self.get_parameter('compress').value)
        self.stop_delay = float(self.get_parameter('stop_delay').value)

        self.proc = None
        self.stopped = False
        self.stop_timer = None

        os.makedirs(self.bag_dir, exist_ok=True)
        self.bag_path = self.next_bag_path()

        self.warn_if_disk_low()
        self.start_recording()

        self.create_subscription(
            Bool, '/stepper/sweep_done', self.sweep_done_callback, latched_qos()
        )

    # ==========================================================
    # Penamaan bag
    # ==========================================================

    def next_index(self):
        """Cari index terbesar yang sudah dipakai di bag_dir, lalu +1."""
        pattern = re.compile(re.escape(self.bag_prefix) + r'_(\d+)')
        used = []
        try:
            entries = os.listdir(self.bag_dir)
        except OSError:
            entries = []

        for name in entries:
            m = pattern.match(name)
            if m:
                used.append(int(m.group(1)))

        return max(used) + 1 if used else 1

    def next_bag_path(self):
        """Contoh hasil: ~/bags/scan_0001_3sweep

        Index selalu max(yang sudah ada) + 1, jadi nama tidak mungkin bentrok
        walau jumlah sweep-nya berbeda-beda.
        """
        name = f'{self.bag_prefix}_{self.next_index():04d}_{self.num_sweeps}sweep'
        return os.path.join(self.bag_dir, name)

    # ==========================================================
    # Recorder
    # ==========================================================

    def supports_topics_flag(self):
        """Jazzy+ pakai `--topics`, Humble pakai topic positional."""
        try:
            out = subprocess.run(
                ['ros2', 'bag', 'record', '--help'],
                capture_output=True, text=True, timeout=30,
            )
            return '--topics' in (out.stdout + out.stderr)
        except (subprocess.SubprocessError, OSError):
            return False

    def build_command(self):
        cmd = [
            'ros2', 'bag', 'record',
            '-s', self.storage,
            '-o', self.bag_path,
            # Recorder jalan sebagai subprocess tanpa terminal interaktif.
            '--disable-keyboard-controls',
        ]

        if self.compress:
            cmd += ['--compression-mode', 'file', '--compression-format', 'zstd']

        if self.supports_topics_flag():
            cmd += ['--topics'] + self.topics
        else:
            cmd += self.topics

        return cmd

    def warn_if_disk_low(self):
        try:
            free_gb = shutil.disk_usage(self.bag_dir).free / 1e9
        except OSError:
            return
        if free_gb < 2.0:
            self.get_logger().warn(
                f'Sisa disk cuma {free_gb:.1f} GB. Rekaman bisa putus di tengah jalan.'
            )

    def start_recording(self):
        cmd = self.build_command()
        self.get_logger().info(f'Mulai merekam ke: {self.bag_path}')
        self.get_logger().info('Perintah: ' + ' '.join(cmd))

        try:
            self.proc = subprocess.Popen(cmd)
        except OSError as exc:
            self.get_logger().error(
                f'Gagal menjalankan ros2 bag record: {exc}. '
                'Rekaman dilewati, mapping tetap jalan.'
            )
            self.proc = None

    def sweep_done_callback(self, msg):
        if not msg.data or self.stopped or self.proc is None:
            return

        self.stopped = True
        self.get_logger().info(
            f'Sweep selesai. Recorder ditutup dalam {self.stop_delay:.1f} detik '
            '(menunggu cloud final)...'
        )
        self.stop_timer = self.create_timer(self.stop_delay, self.on_stop_timer)

    def on_stop_timer(self):
        if self.stop_timer is not None:
            self.stop_timer.cancel()
            self.destroy_timer(self.stop_timer)
            self.stop_timer = None
        self.stop_recording()

    def stop_recording(self):
        """Tutup recorder dengan SIGINT supaya metadata.yaml ditulis lengkap."""
        if self.proc is None or self.proc.poll() is not None:
            self.report()
            return

        self.get_logger().info('Menutup recorder...')
        self.proc.send_signal(signal.SIGINT)

        try:
            self.proc.wait(timeout=60)
        except subprocess.TimeoutExpired:
            self.get_logger().warn('Recorder tidak merespons SIGINT, dipaksa berhenti.')
            self.proc.terminate()
            try:
                self.proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                self.proc.kill()
                self.proc.wait(timeout=10)

        self.report()

    def report(self):
        name = os.path.basename(self.bag_path)
        meta = os.path.join(self.bag_path, 'metadata.yaml')

        if not os.path.exists(meta):
            self.get_logger().error(
                f'metadata.yaml tidak ditemukan di {self.bag_path}. '
                'Bag kemungkinan tidak lengkap.'
            )
            return

        # Tunggu sebentar kalau file mcap masih di-flush ke disk.
        for _ in range(10):
            if dir_size_mb(self.bag_path) > 0:
                break
            time.sleep(0.2)

        self.get_logger().info(
            f'=== REKAMAN TERSIMPAN === {name} '
            f'({dir_size_mb(self.bag_path):.1f} MB)'
        )
        self.get_logger().info(f'Lokasi : {self.bag_path}')
        self.get_logger().info(f'Putar  : ros2 bag play {self.bag_path}')

    def destroy_node(self):
        # Ctrl+C sebelum sweep selesai: tetap tutup rekaman dengan rapi.
        if not self.stopped:
            self.stopped = True
            self.stop_recording()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BagRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
