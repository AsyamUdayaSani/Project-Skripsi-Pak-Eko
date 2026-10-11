import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_msgs.msg import Header, Float32
import numpy as np
import struct
from scipy.spatial.transform import Rotation


class Mapping3DFastLIOScan(Node):
    def __init__(self):
        super().__init__("mapping_3d_fastlio_scan")

        self.servo_buffer = []

        self.create_subscription(
            Float32,
            "/stepper/angle",
            self.servo_callback,
            100
        )
        self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_callback,
            10
        )
        self.cloud_pub = self.create_publisher(
            PointCloud2,
            "/map_3d",
            10
        )

        self.get_logger().info("Mapping3D FAST-LIO Started (Per-Scan Mode)")

    def get_time_sec(self, header=None, clock_time=None):
        if header is not None:
            return header.stamp.sec + header.stamp.nanosec * 1e-9
        if clock_time is not None:
            return clock_time.nanoseconds * 1e-9
        return 0.0

    def servo_callback(self, msg):
        current_time = self.get_time_sec(clock_time=self.get_clock().now())
        angle = float(msg.data)
        self.servo_buffer.append((current_time, angle))

        if len(self.servo_buffer) > 100:
            self.servo_buffer.pop(0)

    def scan_callback(self, msg):
        if len(self.servo_buffer) < 2:
            return

        scan_start_time = self.get_time_sec(header=msg.header)
        time_increment = msg.time_increment
        if time_increment <= 0.0:
            time_increment = (msg.scan_time / len(msg.ranges)) if msg.scan_time > 0.0 else (1.0 / 10.0) / len(msg.ranges)

        servo_times = np.array([item[0] for item in self.servo_buffer], dtype=float)
        servo_angles = np.array([item[1] for item in self.servo_buffer], dtype=float)

        points_to_publish = []
        valid_points = 0
        scan_angle = msg.angle_min

        for i, distance in enumerate(msg.ranges):
            if not (msg.range_min < distance < msg.range_max):
                scan_angle += msg.angle_increment
                continue

            ray_time = scan_start_time + (i * time_increment)
            servo_angle = float(np.interp(ray_time, servo_times, servo_angles))

            x = distance * np.cos(scan_angle)
            y = distance * np.sin(scan_angle)
            z = 0.0

            point = np.array([x, y, z])
            point = Rotation.from_euler("x", -servo_angle).apply(point)

            if i < len(msg.intensities):
                intensity = float(msg.intensities[i])
            else:
                intensity = 0.0

            points_to_publish.append(
                (point[0], point[1], point[2], intensity, 0, float(ray_time - scan_start_time))
            )
            valid_points += 1
            scan_angle += msg.angle_increment

        if valid_points < 10:
            return

        self.publish_cloud(points_to_publish, msg.header.stamp)

    def publish_cloud(self, points, stamp):
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name="ring", offset=16, datatype=PointField.UINT16, count=1),
            PointField(name="time", offset=20, datatype=PointField.FLOAT32, count=1)
        ]

        data = bytearray()
        for point in points:
            data += struct.pack(
                "ffffH2xf",
                point[0], point[1], point[2],
                point[3], int(point[4]), point[5]
            )

        cloud = PointCloud2()
        cloud.header = Header()
        cloud.header.stamp = stamp
        cloud.header.frame_id = "lidar_tilt"
        cloud.height = 1
        cloud.width = len(points)
        cloud.fields = fields
        cloud.is_bigendian = False
        cloud.point_step = 24
        cloud.row_step = cloud.point_step * cloud.width
        cloud.data = bytes(data)
        cloud.is_dense = True

        self.cloud_pub.publish(cloud)


def main():
    rclpy.init()
    node = Mapping3DFastLIOScan()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
