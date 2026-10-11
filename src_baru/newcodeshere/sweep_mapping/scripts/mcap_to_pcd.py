#!/usr/bin/env python3
"""Ekspor PointCloud2 dari bag (.mcap) menjadi file .pcd untuk CloudCompare.

Contoh:
    python3 mcap_to_pcd.py ~/bags/scan_0001_3sweep
    python3 mcap_to_pcd.py ~/bags/scan_0001_3sweep -o ~/hasil.pcd
    python3 mcap_to_pcd.py ~/bags/test_lab_02 --topic /map_3d --ascii

Secara default mengambil pesan TERAKHIR dari /map_3d. Itu memang yang paling
lengkap, karena mapping_3d_sweep mem-publish cloud kumulatif — pesan terakhir
sudah berisi seluruh titik dari semua sweep.
"""

import argparse
import os
import sys

import numpy as np

# Pemetaan datatype PointField -> tipe numpy
PF_TO_NP = {
    1: np.int8, 2: np.uint8, 3: np.int16, 4: np.uint16,
    5: np.int32, 6: np.uint32, 7: np.float32, 8: np.float64,
}
# Pemetaan tipe numpy -> (TYPE, SIZE) untuk header PCD
NP_TO_PCD = {
    np.dtype(np.int8): ('I', 1), np.dtype(np.uint8): ('U', 1),
    np.dtype(np.int16): ('I', 2), np.dtype(np.uint16): ('U', 2),
    np.dtype(np.int32): ('I', 4), np.dtype(np.uint32): ('U', 4),
    np.dtype(np.float32): ('F', 4), np.dtype(np.float64): ('F', 8),
}


def cloud_to_array(msg):
    """PointCloud2 -> numpy structured array, mengikuti offset asli pesannya."""
    names, formats, offsets = [], [], []
    for f in msg.fields:
        if f.datatype not in PF_TO_NP:
            continue
        if f.name in names:          # nama ganda tidak diperbolehkan numpy
            continue
        names.append(f.name)
        formats.append(PF_TO_NP[f.datatype])
        offsets.append(f.offset)

    dtype = np.dtype({
        'names': names,
        'formats': formats,
        'offsets': offsets,
        'itemsize': msg.point_step,
    })
    n = msg.width * msg.height
    return np.frombuffer(bytes(msg.data[:n * msg.point_step]), dtype=dtype, count=n)


def write_pcd(path, points, fields, ascii_mode=False):
    """Tulis file .pcd (biner atau ascii) yang bisa dibuka CloudCompare."""
    sizes, types = [], []
    for name in fields:
        t, s = NP_TO_PCD[points.dtype[name]]
        types.append(t)
        sizes.append(str(s))

    n = points.size
    header = (
        '# .PCD v0.7 - Point Cloud Data file format\n'
        'VERSION 0.7\n'
        f'FIELDS {" ".join(fields)}\n'
        f'SIZE {" ".join(sizes)}\n'
        f'TYPE {" ".join(types)}\n'
        f'COUNT {" ".join("1" for _ in fields)}\n'
        f'WIDTH {n}\n'
        'HEIGHT 1\n'
        'VIEWPOINT 0 0 0 1 0 0 0\n'
        f'POINTS {n}\n'
        f'DATA {"ascii" if ascii_mode else "binary"}\n'
    )

    # Rapatkan ke dtype tanpa padding supaya cocok dengan header PCD.
    packed = np.empty(n, dtype=np.dtype(
        [(name, points.dtype[name]) for name in fields]))
    for name in fields:
        packed[name] = points[name]

    with open(path, 'wb') as fh:
        fh.write(header.encode())
        if ascii_mode:
            np.savetxt(fh, np.column_stack(
                [packed[name] for name in fields]), fmt='%.6f')
        else:
            fh.write(packed.tobytes())


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bag', help='folder bag, mis. ~/bags/scan_0001_3sweep')
    ap.add_argument('-o', '--output', help='file .pcd keluaran')
    ap.add_argument('--topic', default='/map_3d')
    ap.add_argument('--ascii', action='store_true',
                    help='tulis ascii (lebih besar, bisa dibaca teks editor)')
    ap.add_argument('--fields', default='x,y,z,intensity',
                    help='field yang disimpan (default: x,y,z,intensity)')
    ap.add_argument('--index', type=int, default=-1,
                    help='pesan ke berapa (-1 = terakhir, paling lengkap)')
    ap.add_argument('--merge', action='store_true',
                    help='gabungkan SEMUA pesan. Perlu untuk bag lama dari '
                         'mapping_3d.py (cloud per-sweep). JANGAN dipakai untuk '
                         'bag dari mapping_3d_sweep (cloud sudah kumulatif, '
                         'hasilnya jadi dobel).')
    args = ap.parse_args()

    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    bag = os.path.expanduser(args.bag)
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag, storage_id='mcap'),
        rosbag2_py.ConverterOptions('', ''))

    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if args.topic not in type_map:
        sys.exit(f'Topic {args.topic} tidak ada di bag. Yang tersedia: '
                 f'{sorted(type_map)}')

    msgs = []
    while reader.has_next():
        topic, data, _ts = reader.read_next()
        if topic == args.topic:
            msgs.append(data)

    if not msgs:
        sys.exit(f'{args.topic} ada di bag tapi 0 pesan. Tidak ada yang diekspor.')

    msg_type = get_message(type_map[args.topic])

    if args.merge:
        print(f'{args.topic}: {len(msgs)} pesan, MENGGABUNG semuanya')
        chunks = []
        for raw in msgs:
            chunk = cloud_to_array(deserialize_message(raw, msg_type))
            if chunk.size:
                chunks.append(chunk)
        if not chunks:
            sys.exit('Semua pesan kosong, tidak ada yang diekspor.')
        msg = deserialize_message(msgs[-1], msg_type)
        # Prealokasi lalu salin: np.concatenate memampatkan dtype bercelah.
        points = np.empty(sum(c.size for c in chunks), dtype=chunks[0].dtype)
        at = 0
        for chunk in chunks:
            points[at:at + chunk.size] = chunk
            at += chunk.size
    else:
        print(f'{args.topic}: {len(msgs)} pesan, memakai index {args.index}')
        msg = deserialize_message(msgs[args.index], msg_type)
        points = cloud_to_array(msg)

    print(f'frame_id  : {msg.header.frame_id}')
    print(f'point_step: {msg.point_step} byte')
    print(f'field     : {[f.name for f in msg.fields]}')
    print(f'jumlah    : {points.size} titik')

    wanted = [f for f in args.fields.split(',') if f in points.dtype.names]
    missing = [f for f in args.fields.split(',') if f not in points.dtype.names]
    if missing:
        print(f'(dilewati, tidak ada di cloud: {missing})')
    if not all(k in wanted for k in ('x', 'y', 'z')):
        sys.exit('Cloud tidak punya x/y/z, tidak bisa diekspor.')

    # Buang titik NaN/inf supaya CloudCompare tidak protes.
    finite = np.ones(points.size, dtype=bool)
    for k in ('x', 'y', 'z'):
        finite &= np.isfinite(points[k])
    dropped = points.size - int(finite.sum())
    if dropped:
        print(f'membuang {dropped} titik NaN/inf')
    points = points[finite]

    out = args.output or (os.path.basename(os.path.normpath(bag)) + '.pcd')
    out = os.path.expanduser(out)
    write_pcd(out, points, wanted, ascii_mode=args.ascii)

    print(f'\nTersimpan: {out} ({os.path.getsize(out)/1e6:.1f} MB, '
          f'{points.size} titik, field {wanted})')
    print('Buka di CloudCompare: File > Open, pilih file .pcd ini.')


if __name__ == '__main__':
    main()
