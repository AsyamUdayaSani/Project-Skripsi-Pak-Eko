import rclpy
from rclpy.node import Node

from sensor_msgs.msg import (
    LaserScan,
    PointCloud2,
    PointField
)

from std_msgs.msg import (
    Header,
    Float32
)

import numpy as np
import struct
from scipy.spatial.transform import Rotation


class Mapping3DFastLIO(Node):

    def __init__(self):
        super().__init__("mapping_3d_fastlio")

        # Variabel state
        self.latest_angle = None
        self.last_stepper_angle = None
        self.current_sweep_points = []
        self.sweep_start_time = None

        # Subscriber stepper
        self.create_subscription(
            Float32,
            "/stepper/angle",
            self.servo_callback,
            100
        )

        # Subscriber lidar
        self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_callback,
            10
        )

        # Publisher cloud
        self.cloud_pub = self.create_publisher(
            PointCloud2,
            "/map_3d",
            10
        )

        self.get_logger().info("Mapping3D FAST-LIO Started (Full Sweep Mode)")

    # ======================================================
    # Utility
    # ======================================================

    def get_time_sec(self, header=None, clock_time=None):
        if header is not None:
            return header.stamp.sec + header.stamp.nanosec * 1e-9
        if clock_time is not None:
            return clock_time.nanoseconds * 1e-9
        return 0.0

    # ======================================================
    # Stepper Callback
    # ======================================================

    def servo_callback(self, msg):
        angle = msg.data

        if self.last_stepper_angle is not None:
            # Deteksi ketika stepper kembali ke 0 (satu putaran/sweep selesai)
            if self.last_stepper_angle > 5.5 and angle < 0.5:
                self.publish_current_sweep()

        self.last_stepper_angle = angle
        self.latest_angle = angle

    # ======================================================
    # LiDAR Callback
    # ======================================================

    def scan_callback(self, msg):
        if self.latest_angle is None:
            return

        time_increment = msg.time_increment
        if time_increment <= 0.0:
            time_increment = msg.scan_time / len(msg.ranges)

        scan_angle = msg.angle_min
        servo_angle = self.latest_angle
        rotation = Rotation.from_euler("x", -servo_angle)

        scan_start_time = self.get_time_sec(header=msg.header)

        # Menandai awal dari satu sweep (penting untuk perhitungan waktu IMU)
        if not self.current_sweep_points:
            self.sweep_start_time = scan_start_time

        # Hitung jarak waktu scan saat ini dari scan pertama di sweep ini
        offset_base = scan_start_time - self.sweep_start_time

        valid_points = 0
        for i, distance in enumerate(msg.ranges):
            if not (msg.range_min < distance < msg.range_max):
                scan_angle += msg.angle_increment
                continue

            # Waktu (offset) harus dihitung dari AWAL SWEEP, bukan awal scan
            offset_time = offset_base + (i * time_increment)

            x = distance * np.cos(scan_angle)
            y = distance * np.sin(scan_angle)
            z = 0.0

            point = np.array([x, y, z])
            point = rotation.apply(point)

            if i < len(msg.intensities):
                intensity = float(msg.intensities[i])
            else:
                intensity = 0.0

            self.current_sweep_points.append(
                (
                    point[0],
                    point[1],
                    point[2],
                    intensity,
                    0,                  # ring selalu 0
                    float(offset_time)  # waktu relatif dalam detik dari awal sweep
                )
            )

            valid_points += 1
            scan_angle += msg.angle_increment

    # ======================================================
    # Publish PointCloud
    # ======================================================

    def publish_current_sweep(self):
        # Jika point terlalu sedikit (misal belum ada data valid), batalkan
        if len(self.current_sweep_points) < 100:
            self.current_sweep_points.clear()
            self.sweep_start_time = None
            return

        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name="ring", offset=16, datatype=PointField.UINT16, count=1),
            PointField(name="time", offset=20, datatype=PointField.FLOAT32, count=1)
        ]

        data = bytearray()
        for point in self.current_sweep_points:
            data += struct.pack(
                "ffffH2xf",
                float(point[0]), float(point[1]), float(point[2]),
                float(point[3]), int(point[4]), float(point[5])
            )

        cloud = PointCloud2()
        cloud.header = Header()
        
        # Waktu stamp PointCloud HARUS sama dengan waktu dimulainya sweep
        # agar FAST-LIO bisa mencocokkan waktu point dengan IMU buffer
        if self.sweep_start_time is not None:
            sec = int(self.sweep_start_time)
            nanosec = int((self.sweep_start_time - sec) * 1e9)
            cloud.header.stamp.sec = sec
            cloud.header.stamp.nanosec = nanosec
        else:
            cloud.header.stamp = self.get_clock().now().to_msg()

        cloud.header.frame_id = "lidar_tilt"
        cloud.height = 1
        cloud.width = len(self.current_sweep_points)
        cloud.fields = fields
        cloud.is_bigendian = False
        cloud.point_step = 24
        cloud.row_step = cloud.point_step * cloud.width
        cloud.data = bytes(data)
        cloud.is_dense = True

        last_offset = self.current_sweep_points[-1][5]

        self.get_logger().info(
            f"Published sweep: {cloud.width} points. "
            f"Total sweep duration: {last_offset*1000:.2f} ms"
        )

        self.cloud_pub.publish(cloud)

        # Bersihkan untuk sweep berikutnya
        self.current_sweep_points.clear()
        self.sweep_start_time = None

    # ======================================================
    # Shutdown
    # ======================================================

    def destroy_node(self):
        self.get_logger().info("Mapping3D FAST-LIO stopped")
        super().destroy_node()


def main():
    rclpy.init()
    node = Mapping3DFastLIO()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()