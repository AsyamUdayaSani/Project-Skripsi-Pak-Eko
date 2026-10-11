#!/usr/bin/env python3
"""Diagnosa ketidaksejajaran sumbu putar stepper dari satu rekaman.

Satu sweep 360 derajat mengukur tiap arah DUA KALI: titik pada sudut sinar
theta dengan stepper `a` mendarat di tempat yang sama dengan sinar `-theta`
pada stepper `a+180`. Jadi paruh pertama dan paruh kedua sweep seharusnya
menghasilkan peta yang identik.

Kalau keduanya tidak bertumpuk, penyebabnya BUKAN tanda rotasi, bukan
tilt_offset, bukan latensi waktu - ketiganya memutar kedua paruh bersama-sama
sehingga tetap saling cocok. Yang bisa memisahkan keduanya hanya kesalahan pada
SUMBU PUTARNYA sendiri.

Skrip ini memecah rekaman jadi dua paruh, mengukur selisihnya, lalu memisahkan
dua penyebab yang mungkin:

  * arah sumbu tidak sejajar  -> selisih MEMBESAR seiring jarak dari sumbu
  * sumbu bergeser dari pusat -> selisih TETAP berapa pun jaraknya

Pemakaian:
    python3 diagnosa_sumbu.py ~/bags/scan_0021_1sweep
    python3 diagnosa_sumbu.py rekaman.mcap --out-dir ~/hasil
"""

import argparse
import math
import os

import numpy as np
import rclpy
import rosbag2_py
from rclpy.serialization import deserialize_message
from scipy.spatial import cKDTree
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32, Int32

from sweep_mapping.cloud_utils import POINT_DTYPE, merge_chunks
from sweep_mapping.mapping_3d_sweep1 import Mapping3DSweep

TIPE_PESAN = {
    'sensor_msgs/msg/LaserScan': LaserScan,
    'std_msgs/msg/Float32': Float32,
    'std_msgs/msg/Int32': Int32,
    'std_msgs/msg/Bool': Bool,
}

# Batas bawah yang masuk akal untuk jarak tetangga terdekat: dua titik dari
# pemindaian berbeda tidak akan pernah jatuh persis di tempat yang sama, jadi
# selisih sekecil jarak antar-titik bukan bukti kesalahan apa pun.
LANTAI_WAJAR_M = 0.01


class Fusi:
    """Fusi scan + sudut stepper, sambil MENYIMPAN sudut stepper tiap titik.

    Node produksi membuang sudut itu setelah dipakai, padahal justru itu yang
    dibutuhkan untuk memecah cloud jadi dua paruh. Rumusnya disalin persis dari
    mapping_3d_sweep.py, lalu dibuktikan sama lewat swauji di bawah.
    """

    def __init__(self, rotation_sign=-1.0, tilt_offset=0.0):
        self.rotation_sign = rotation_sign
        self.tilt_offset = tilt_offset
        self.servo_times = []
        self.servo_angles = []
        self.chunks = []
        self.sudut = []
        self.total = 0
        self.done = False
        self.t = 0.0

    def servo(self, msg):
        self.servo_times.append(self.t)
        self.servo_angles.append(float(msg.data))
        if len(self.servo_times) > 100:
            self.servo_times.pop(0)
            self.servo_angles.pop(0)

    def sweep_done(self, msg):
        if msg.data:
            self.done = True

    def scan(self, msg):
        if self.done or len(self.servo_times) < 2:
            return

        ranges = np.asarray(msg.ranges, dtype=np.float64)
        if ranges.size == 0:
            return

        with np.errstate(invalid='ignore'):
            valid = ((ranges > msg.range_min) & (ranges < msg.range_max)
                     & np.isfinite(ranges))
        keep = np.flatnonzero(valid)
        if keep.size == 0:
            return

        ti = msg.time_increment
        if ti <= 0.0:
            ti = (1.0 / 10.0) / ranges.size

        t0 = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        offset = keep * ti
        ray_t = t0 + offset

        servo = np.interp(ray_t, np.asarray(self.servo_times),
                          np.asarray(self.servo_angles))

        r = ranges[keep]
        sa = msg.angle_min + keep * msg.angle_increment
        x = r * np.cos(sa)
        y = r * np.sin(sa)
        a = self.rotation_sign * servo + self.tilt_offset

        inten = np.asarray(msg.intensities, dtype=np.float32)
        if inten.size < ranges.size:
            pad = np.zeros(ranges.size, dtype=np.float32)
            pad[:inten.size] = inten
            inten = pad

        chunk = np.empty(keep.size, dtype=POINT_DTYPE)
        chunk['x'] = x
        chunk['y'] = y * np.cos(a)
        chunk['z'] = y * np.sin(a)
        chunk['intensity'] = inten[keep]
        chunk['ring'] = 0
        chunk['time'] = offset

        self.chunks.append(chunk)
        # Sudut stepper mentah (belum dikali tanda), dibungkus ke 0..2pi.
        self.sudut.append(np.mod(servo, 2.0 * math.pi))
        self.total += keep.size

    def hasil(self):
        if self.total == 0:
            return None, None
        return merge_chunks(self.chunks, self.total)[0], np.concatenate(self.sudut)


def tulis_ply(path, titik):
    n = titik.size
    kepala = ('ply\nformat binary_little_endian 1.0\n'
              f'element vertex {n}\n'
              'property float x\nproperty float y\nproperty float z\n'
              'property float intensity\nend_header\n')
    keluar = np.empty(n, dtype=[('x', '<f4'), ('y', '<f4'),
                                ('z', '<f4'), ('intensity', '<f4')])
    for k in ('x', 'y', 'z', 'intensity'):
        keluar[k] = titik[k]
    with open(path, 'wb') as f:
        f.write(kepala.encode('ascii'))
        f.write(keluar.tobytes())
    return n


def xyz(titik):
    return np.stack([titik['x'], titik['y'], titik['z']], axis=1).astype(np.float64)


def main():
    ap = argparse.ArgumentParser(
        description='Ukur ketidaksejajaran sumbu putar dengan membandingkan '
                    'paruh pertama dan paruh kedua sweep.')
    ap.add_argument('bag')
    ap.add_argument('--out-dir', default='.')
    ap.add_argument('--sampel', type=int, default=60000,
                    help='Jumlah titik paruh B yang diuji (bawaan 60000). '
                         'Menaikkannya lebih teliti tapi lebih lambat.')
    args = ap.parse_args()
    os.makedirs(args.out_dir, exist_ok=True)

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=args.bag, storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    tipe = {t.name: t.type for t in reader.get_all_topics_and_types()}

    rclpy.init()
    node = Mapping3DSweep()          # acuan, memakai kode produksi apa adanya
    node.publish_every_sweep = False
    jam = {'t': 0.0}
    node.now_sec = lambda: jam['t']

    fusi = Fusi(rotation_sign=node.rotation_sign, tilt_offset=node.tilt_offset)

    while reader.has_next():
        topic, data, ts = reader.read_next()
        if topic not in ('/scan', '/stepper/angle', '/stepper/sweep_done',
                         '/stepper/sweep_count'):
            continue
        kelas = TIPE_PESAN.get(tipe.get(topic))
        if kelas is None:
            continue
        msg = deserialize_message(data, kelas)
        jam['t'] = ts * 1e-9
        fusi.t = jam['t']

        if topic == '/scan':
            node.scan_callback(msg)
            fusi.scan(msg)
        elif topic == '/stepper/angle':
            node.servo_callback(msg)
            fusi.servo(msg)
        elif topic == '/stepper/sweep_done':
            node.sweep_done_callback(msg)
            fusi.sweep_done(msg)
        elif topic == '/stepper/sweep_count':
            node.sweep_count_callback(msg)

    titik, sudut = fusi.hasil()
    if titik is None:
        raise SystemExit('Tidak ada titik terkumpul. Cek isi bag.')

    # Swauji: pastikan fusi di skrip ini identik dengan node produksi.
    acuan = merge_chunks(node.chunks, node.total_points)[0]
    assert acuan.size == titik.size, 'jumlah titik berbeda dari node produksi'
    beda = np.max(np.abs(xyz(acuan) - xyz(titik)))
    assert beda < 1e-6, f'geometri menyimpang dari node produksi ({beda:.2e} m)'

    A = sudut < math.pi
    B = ~A
    tA, tB = titik[A], titik[B]

    print()
    print('=' * 64)
    print('  PEMBAGIAN SWEEP')
    print('=' * 64)
    print(f'  swauji terhadap mapping_3d_sweep : cocok (beda {beda:.1e} m)')
    print(f'  paruh A (stepper   0-180 deg)    : {tA.size:>8d} titik')
    print(f'  paruh B (stepper 180-360 deg)    : {tB.size:>8d} titik')

    if tA.size < 1000 or tB.size < 1000:
        raise SystemExit('\nSalah satu paruh nyaris kosong - sweep mungkin '
                         'belum genap 360 derajat. Diagnosa dilewati.')

    pA, pB = xyz(tA), xyz(tB)
    pohon = cKDTree(pA)

    if pB.shape[0] > args.sampel:
        idx = np.random.default_rng(0).choice(pB.shape[0], args.sampel, replace=False)
        contoh = pB[idx]
    else:
        contoh = pB

    d, _ = pohon.query(contoh, k=1)
    # Jarak dari sumbu putar (sumbu X): hanya komponen y dan z yang berputar.
    r = np.hypot(contoh[:, 1], contoh[:, 2])

    print()
    print('=' * 64)
    print('  SELISIH ANTAR PARUH')
    print('=' * 64)
    print(f'  median   {np.median(d) * 100:7.2f} cm')
    print(f'  rata2    {d.mean() * 100:7.2f} cm')
    print(f'  p95      {np.percentile(d, 95) * 100:7.2f} cm')

    if np.median(d) < LANTAI_WAJAR_M:
        print()
        print('  -> Kedua paruh sudah bertumpuk rapi. Sumbu putarnya sehat;')
        print('     tidak ada yang perlu dikalibrasi.')

    # Pisahkan penyebab: apakah selisih membesar seiring jarak dari sumbu?
    print()
    print('=' * 64)
    print('  SELISIH MENURUT JARAK DARI SUMBU PUTAR')
    print('=' * 64)
    tepi = np.percentile(r, [0, 20, 40, 60, 80, 100])
    pusat, nilai = [], []
    print(f'  {"jarak dari sumbu":>22s}   {"selisih median":>14s}')
    for i in range(5):
        m = (r >= tepi[i]) & (r < tepi[i + 1] if i < 4 else r <= tepi[i + 1])
        if m.sum() < 50:
            continue
        rm, dm = np.median(r[m]), np.median(d[m])
        pusat.append(rm)
        nilai.append(dm)
        print(f'  {tepi[i]:5.1f} - {tepi[i+1]:5.1f} m        {dm * 100:8.2f} cm')

    if len(pusat) >= 3:
        kemiringan, potongan = np.polyfit(np.array(pusat), np.array(nilai), 1)
        sudut_deg = math.degrees(math.atan(abs(kemiringan)))
        print()
        print('=' * 64)
        print('  KESIMPULAN')
        print('=' * 64)
        print(f'  selisih tumbuh {kemiringan * 100:+.3f} cm per meter jarak')
        print(f'  selisih pada jarak nol (potongan) {potongan * 100:+.2f} cm')
        print()
        if abs(kemiringan) > 0.005 and abs(kemiringan) * 2 > abs(potongan):
            print('  -> ARAH SUMBU PUTAR TIDAK SEJAJAR.')
            print('     Selisihnya membesar seiring jarak - ciri khas kesalahan')
            print('     sudut, bukan pergeseran. Tembok jauh paling terlihat miring.')
            print()
            print(f'     Kedua paruh saling meleset setara {sudut_deg:.2f} deg.')
            print('     CATATAN: itu selisih ANTAR PARUH, bukan langsung nilai')
            print('     koreksinya. Kesalahan sumbu menumpuk berbeda-beda di tiap')
            print('     sudut stepper, jadi koreksi yang dibutuhkan ada pada orde')
            print(f'     ini - kira-kira {sudut_deg/2:.2f} sampai {sudut_deg:.2f} deg.')
            print('     Nilai pastinya dicari dengan menyetel sampai kedua paruh')
            print('     bertumpuk, bukan dihitung langsung dari angka ini.')
        elif abs(potongan) > 0.02:
            print(f'  -> SUMBU PUTAR BERGESER dari pusat LiDAR, kira-kira '
                  f'{abs(potongan) / 2 * 100:.1f} cm.')
            print('     Selisihnya tetap berapa pun jaraknya - ciri pergeseran,')
            print('     bukan sudut. Tembok tampak ganda sejajar, tidak miring.')
        else:
            print('  -> Tidak ada pola yang menonjol. Selisihnya kecil dan')
            print('     tersebar merata; kemungkinan hanya kerapatan titik.')
        print()
        print('  Kalau selisihnya justru MEMBESAR ke akhir sweep dan LiDAR tidak')
        print('  kembali tepat ke 0 derajat, tersangkanya gear ratio atau motor')
        print('  slip - itu di luar jangkauan skrip ini. Uji dengan menempel')
        print('  selotip penanda lalu menjalankan sweeps:=5.')

    nama = os.path.splitext(os.path.basename(args.bag.rstrip('/')))[0]
    p1 = os.path.join(args.out_dir, f'{nama}_paruhA.ply')
    p2 = os.path.join(args.out_dir, f'{nama}_paruhB.ply')
    n1, n2 = tulis_ply(p1, tA), tulis_ply(p2, tB)

    print()
    print('=' * 64)
    print('  KELUARAN')
    print('=' * 64)
    print(f'  {p1}  ({n1} titik)')
    print(f'  {p2}  ({n2} titik)')
    print()
    print('  Buka keduanya bersamaan di CloudCompare dengan warna berbeda.')
    print('  Kalau sumbunya sehat, keduanya menempel jadi satu. Kalau miring,')
    print('  bedanya akan langsung terlihat pada tembok yang jauh.')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
