#!/usr/bin/env python3
"""Bandingkan hasil mapping DENGAN dan TANPA koreksi IMU atas satu bag yang sama.

Kenapa dari bag, bukan dua scan terpisah: dua scan tidak akan pernah mengalami
goyangan yang sama, jadi bedanya tidak bisa disebut berasal dari koreksi IMU.
Dengan memutar ulang satu rekaman lewat dua node, masukannya identik dan
selisihnya murni dari koreksinya.

Pemakaian:
    python3 bandingkan_imu.py ~/bags/scan_0021_1sweep
    python3 bandingkan_imu.py rekaman.mcap --out-dir ~/hasil

Keluarannya dua berkas PLY (buka berdampingan di CloudCompare) plus ringkasan
angka di terminal.

CATATAN WAKTU: skrip ini memberi node waktu terima dari rekaman, bukan jam
dinding. Itu sebabnya `ros2 bag play` + `use_sim_time` tidak diperlukan - dan
hasilnya lebih deterministik daripada memutar ulang lewat graf ROS.
"""

import argparse
import math
import os
import sys

import numpy as np
import rclpy
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Bool, Float32, Int32

from sweep_mapping.cloud_utils import merge_chunks
from sweep_mapping.mapping_3d_sweep1 import Mapping3DSweep
from sweep_mappingimu.mapping_3d_imu import Mapping3DImu, quat_to_rpy

TIPE_PESAN = {
    'sensor_msgs/msg/LaserScan': LaserScan,
    'sensor_msgs/msg/Imu': Imu,
    'std_msgs/msg/Float32': Float32,
    'std_msgs/msg/Int32': Int32,
    'std_msgs/msg/Bool': Bool,
}

TOPIC_DIPAKAI = (
    '/scan',
    '/imu/data_raw',
    '/stepper/angle',
    '/stepper/sweep_count',
    '/stepper/sweep_done',
)


class JamRekaman:
    """Pengganti now_sec() pada node, mengembalikan waktu dari rekaman.

    Node aslinya memakai jam dinding untuk mencap sudut stepper dan IMU,
    sementara sinar LiDAR dicap dengan header.stamp dari driver. Saat diputar
    ulang, jam dinding sudah tidak ada hubungannya dengan rekaman - maka waktu
    terima dari bag yang dipakai, meniru perilaku aslinya saat rig berjalan.
    """

    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


def buka_bag(path):
    """Kembalikan reader untuk berkas .mcap atau folder bag."""
    if os.path.isdir(path):
        storage_id = 'mcap'
        berkas = [f for f in os.listdir(path) if f.endswith('.mcap')]
        if not berkas:
            raise SystemExit(f'Tidak ada berkas .mcap di dalam {path}')
        uri = path
    else:
        storage_id = 'mcap'
        uri = path

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=uri, storage_id=storage_id),
        rosbag2_py.ConverterOptions('', ''),
    )
    return reader


def tulis_ply(path, titik):
    """Tulis PLY biner little-endian: x, y, z, intensity.

    Tiap titik ditulis TEPAT SEKALI. (Pernah ada bug di skrip lain yang menulis
    tiap titik dua kali sehingga jumlah titiknya menyesatkan - jangan terulang.)
    """
    n = titik.size
    kepala = (
        'ply\n'
        'format binary_little_endian 1.0\n'
        f'element vertex {n}\n'
        'property float x\n'
        'property float y\n'
        'property float z\n'
        'property float intensity\n'
        'end_header\n'
    )

    keluar = np.empty(n, dtype=[('x', '<f4'), ('y', '<f4'),
                                ('z', '<f4'), ('intensity', '<f4')])
    keluar['x'] = titik['x']
    keluar['y'] = titik['y']
    keluar['z'] = titik['z']
    keluar['intensity'] = titik['intensity']

    with open(path, 'wb') as f:
        f.write(kepala.encode('ascii'))
        f.write(keluar.tobytes())

    return n


def ambil_titik(node):
    if node.total_points == 0:
        return None
    return merge_chunks(node.chunks, node.total_points)[0]


def main():
    ap = argparse.ArgumentParser(
        description='Bandingkan mapping dengan dan tanpa koreksi IMU '
                    'atas satu rekaman yang sama.')
    ap.add_argument('bag', help='Berkas .mcap atau folder bag')
    ap.add_argument('--out-dir', default='.',
                    help='Folder keluaran PLY (bawaan: folder sekarang)')
    ap.add_argument('--use-yaw', action='store_true',
                    help='Ikut mengoreksi yaw. Mati secara bawaan karena yaw '
                         'berasal dari magnetometer.')
    ap.add_argument('--roll-offset', type=float, default=0.0,
                    help='Bias pemasangan IMU sumbu roll, derajat.')
    ap.add_argument('--pitch-offset', type=float, default=0.0,
                    help='Bias pemasangan IMU sumbu pitch, derajat.')
    args = ap.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)

    reader = buka_bag(args.bag)
    tipe_topic = {t.name: t.type for t in reader.get_all_topics_and_types()}

    hilang = [t for t in TOPIC_DIPAKAI if t not in tipe_topic]
    if hilang:
        print(f'Topic tidak ada di bag ini: {", ".join(hilang)}')
        if '/imu/data_raw' in hilang:
            raise SystemExit('Tanpa /imu/data_raw tidak ada yang bisa dibandingkan.')

    rclpy.init()

    jam = JamRekaman()

    node_polos = Mapping3DSweep()
    node_imu = Mapping3DImu()
    for n in (node_polos, node_imu):
        n.now_sec = jam
        # Publish per sweep tidak berguna di sini dan hanya memperlambat.
        n.publish_every_sweep = False

    node_imu.roll_offset = math.radians(args.roll_offset)
    node_imu.pitch_offset = math.radians(args.pitch_offset)
    node_imu.use_yaw = args.use_yaw

    # Riwayat orientasi mentah, untuk melaporkan seberapa besar goyangannya.
    roll_semua = []
    pitch_semua = []

    jumlah = {t: 0 for t in TOPIC_DIPAKAI}

    while reader.has_next():
        topic, data, stamp_ns = reader.read_next()
        if topic not in TOPIC_DIPAKAI:
            continue

        kelas = TIPE_PESAN.get(tipe_topic[topic])
        if kelas is None:
            continue

        msg = deserialize_message(data, kelas)
        jam.t = stamp_ns * 1e-9
        jumlah[topic] += 1

        if topic == '/scan':
            node_polos.scan_callback(msg)
            node_imu.scan_callback(msg)
        elif topic == '/stepper/angle':
            node_polos.servo_callback(msg)
            node_imu.servo_callback(msg)
        elif topic == '/imu/data_raw':
            node_imu.imu_callback(msg)
            q = msg.orientation
            r, p, _ = quat_to_rpy(q.x, q.y, q.z, q.w)
            roll_semua.append(r)
            pitch_semua.append(p)
        elif topic == '/stepper/sweep_count':
            node_polos.sweep_count_callback(msg)
            node_imu.sweep_count_callback(msg)
        elif topic == '/stepper/sweep_done':
            node_polos.sweep_done_callback(msg)
            node_imu.sweep_done_callback(msg)

    titik_polos = ambil_titik(node_polos)
    titik_imu = ambil_titik(node_imu)

    print()
    print('=' * 62)
    print('  MASUKAN')
    print('=' * 62)
    for t in TOPIC_DIPAKAI:
        print(f'  {t:24s} {jumlah[t]:>8d} pesan')

    if titik_polos is None or titik_imu is None:
        raise SystemExit('\nTidak ada titik yang terkumpul. '
                         'Cek apakah bag berisi /scan dan /stepper/angle.')

    print()
    print('=' * 62)
    print('  GOYANGAN RIG SELAMA REKAMAN')
    print('=' * 62)
    if roll_semua:
        roll = np.degrees(np.asarray(roll_semua))
        pitch = np.degrees(np.asarray(pitch_semua))
        print(f'  roll   rentang {roll.min():+7.2f} .. {roll.max():+7.2f} deg'
              f'   (simpangan baku {roll.std():.2f})')
        print(f'  pitch  rentang {pitch.min():+7.2f} .. {pitch.max():+7.2f} deg'
              f'   (simpangan baku {pitch.std():.2f})')
        goyang = max(roll.max() - roll.min(), pitch.max() - pitch.min())
        if goyang < 0.5:
            print('  -> Rig praktis diam. Koreksi IMU memang tidak akan banyak')
            print('     mengubah apa pun; itu hasil yang benar, bukan kegagalan.')
    else:
        print('  (tidak ada pesan IMU)')

    print()
    print('=' * 62)
    print('  SELISIH HASIL')
    print('=' * 62)
    print(f'  titik tanpa IMU : {titik_polos.size:>9d}')
    print(f'  titik dengan IMU: {titik_imu.size:>9d}')

    if node_imu.uncorrected_points:
        print(f'  PERINGATAN: {node_imu.uncorrected_points} titik masuk TANPA '
              'koreksi (IMU belum siap saat itu)')

    if titik_polos.size != titik_imu.size:
        print('  Jumlah titik berbeda - perbandingan per titik dilewati.')
    else:
        d = np.sqrt(
            (titik_imu['x'] - titik_polos['x']) ** 2
            + (titik_imu['y'] - titik_polos['y']) ** 2
            + (titik_imu['z'] - titik_polos['z']) ** 2
        )
        print()
        print('  Seberapa jauh koreksi menggeser tiap titik:')
        print(f'    rata-rata  {d.mean():8.3f} m')
        print(f'    median     {np.median(d):8.3f} m')
        print(f'    persentil 95 {np.percentile(d, 95):6.3f} m')
        print(f'    maksimum   {d.max():8.3f} m')
        print()
        print('  Angka besar = rig banyak bergoyang dan koreksinya bekerja keras.')
        print('  Angka ~0    = rig diam; kedua hasil memang seharusnya sama.')

    nama = os.path.splitext(os.path.basename(args.bag.rstrip('/')))[0]
    p1 = os.path.join(args.out_dir, f'{nama}_TANPA_imu.ply')
    p2 = os.path.join(args.out_dir, f'{nama}_DENGAN_imu.ply')
    n1 = tulis_ply(p1, titik_polos)
    n2 = tulis_ply(p2, titik_imu)

    print()
    print('=' * 62)
    print('  KELUARAN')
    print('=' * 62)
    print(f'  {p1}  ({n1} titik)')
    print(f'  {p2}  ({n2} titik)')
    print()
    print('  Buka keduanya di CloudCompare. Yang dinilai: ketebalan dinding dan')
    print('  lantai. Kalau koreksinya bekerja, versi DENGAN_imu lebih tipis dan')
    print('  tajam. Kalau rig tidak bergoyang, keduanya akan mirip - dan itu')
    print('  memang jawaban yang benar.')

    for n in (node_polos, node_imu):
        n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
