"""Bagian pointcloud yang dipakai bersama oleh node mapping.

Dipisah ke sini supaya `mapping_3d_sweep` (tanpa IMU) dan `mapping_3d_imu`
(dengan IMU) memakai tata letak titik yang sama persis. Kalau tiap node punya
salinan POINT_DTYPE sendiri, cepat atau lambat salah satunya akan menyimpang
dan cloud-nya jadi kacau di viewer tanpa pesan error apa pun.
"""

import numpy as np
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

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
    """QoS latched: subscriber yang telat nyala tetap dapat pesan terakhir."""
    return QoSProfile(
        depth=1,
        history=QoSHistoryPolicy.KEEP_LAST,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    )


def merge_chunks(chunks, total_points):
    """Gabungkan daftar chunk jadi satu blok kontinu.

    JANGAN ganti ini dengan np.concatenate. Untuk dtype bercelah (padding 2 byte
    setelah 'ring'), concatenate memampatkan itemsize 24 -> 22 sehingga data
    tidak lagi cocok dengan point_step dan cloud jadi kacau di viewer.
    """
    if len(chunks) <= 1:
        return chunks

    merged = np.empty(total_points, dtype=POINT_DTYPE)
    offset = 0
    for chunk in chunks:
        merged[offset:offset + chunk.size] = chunk
        offset += chunk.size
    return [merged]


def make_cloud(points, frame_id, stamp):
    """Bungkus array terstruktur POINT_DTYPE jadi pesan PointCloud2."""
    assert points.dtype.itemsize == 24, 'padding titik hilang'

    cloud = PointCloud2()
    cloud.header = Header()
    cloud.header.stamp = stamp
    cloud.header.frame_id = frame_id
    cloud.height = 1
    cloud.width = points.size
    cloud.fields = POINT_FIELDS
    cloud.is_bigendian = False
    cloud.point_step = POINT_DTYPE.itemsize
    cloud.row_step = cloud.point_step * cloud.width
    cloud.data = points.tobytes()
    cloud.is_dense = True
    return cloud
