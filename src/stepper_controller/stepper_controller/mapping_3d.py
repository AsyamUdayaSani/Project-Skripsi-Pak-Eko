import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan, PointCloud2, PointField, Imu
from std_msgs.msg import Header, Float32

import numpy as np
import struct
from scipy.spatial.transform import Rotation


class Mapping3D(Node):

    def __init__(self):
        super().__init__('mapping_3d')

        self.servo_buffer = []
        self.latest_imu = None
        self.current_sweep_points = []
        self.last_stepper_angle = None

        # Subscribers
        self.create_subscription(
            Float32,
            '/stepper/angle',
            self.servo_callback,
            50
        )

        self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.create_subscription(
            Imu,
            '/imu/data_raw',
            self.imu_callback,
            50
        )

        # Publisher
        self.map_pub = self.create_publisher(
            PointCloud2,
            '/map_3d',
            10
        )

        self.get_logger().info(
            'Mapping3D V2 started (LiDAR + Stepper + IMU)'
        )

    # ==========================================================
    # Utility
    # ==========================================================

    def get_time_sec(self, header=None, clock_time=None):
        if header is not None:
            return header.stamp.sec + header.stamp.nanosec * 1e-9

        if clock_time is not None:
            return clock_time.nanoseconds * 1e-9

        return 0.0

    # ==========================================================
    # IMU Callback
    # ==========================================================

    def imu_callback(self, msg):
        self.latest_imu = msg

    # ==========================================================
    # Stepper Callback
    # ==========================================================

    def servo_callback(self, msg):

        current_time = self.get_time_sec(
            clock_time=self.get_clock().now()
        )

        angle = msg.data

        # Deteksi satu putaran selesai
        if self.last_stepper_angle is not None:
            if self.last_stepper_angle > 5.5 and angle < 0.5:
                self.publish_current_sweep()

        self.last_stepper_angle = angle

        self.servo_buffer.append(
            (current_time, angle)
        )

        if len(self.servo_buffer) > 100:
            self.servo_buffer.pop(0)

    # ==========================================================
    # LiDAR Callback
    # ==========================================================

    def scan_callback(self, msg):

        if len(self.servo_buffer) < 2:
            return

        if self.latest_imu is None:
            return

        q = self.latest_imu.orientation

        imu_rotation = Rotation.from_quat([
            q.x,
            q.y,
            q.z,
            q.w
        ])

        servo_times = np.array([
            item[0] for item in self.servo_buffer
        ])

        servo_angles = np.array([
            item[1] for item in self.servo_buffer
        ])

        scan_start_time = self.get_time_sec(header=msg.header)

        time_increment = msg.time_increment

        if time_increment <= 0.0:
            time_increment = (1.0 / 10.0) / len(msg.ranges)

        scan_angle = msg.angle_min

        for i, distance in enumerate(msg.ranges):

            if msg.range_min < distance < msg.range_max:

                ray_time = scan_start_time + i * time_increment

                servo_angle = np.interp(
                    ray_time,
                    servo_times,
                    servo_angles
                )

                # Titik pada frame LiDAR
                x = distance * np.cos(scan_angle)
                y = distance * np.sin(scan_angle)
                z = 0.0

                point = np.array([x, y, z])

                # Rotasi Stepper
                stepper_rotation = Rotation.from_euler(
                    'x',
                    -servo_angle
                )

                # Rotasi IMU
                rotation = imu_rotation * stepper_rotation

                point_rotated = rotation.apply(point)

                self.current_sweep_points.append(point_rotated)

            scan_angle += msg.angle_increment

    # ==========================================================
    # Publish PointCloud
    # ==========================================================

    def publish_current_sweep(self):

        if len(self.current_sweep_points) < 100:
            return

        fields = [
            PointField(
                name='x',
                offset=0,
                datatype=PointField.FLOAT32,
                count=1
            ),
            PointField(
                name='y',
                offset=4,
                datatype=PointField.FLOAT32,
                count=1
            ),
            PointField(
                name='z',
                offset=8,
                datatype=PointField.FLOAT32,
                count=1
            )
        ]

        data = bytearray()

        for point in self.current_sweep_points:
            data += struct.pack(
                'fff',
                float(point[0]),
                float(point[1]),
                float(point[2])
            )

        cloud = PointCloud2()

        cloud.header = Header()
        cloud.header.stamp = self.get_clock().now().to_msg()

        # Frame LiDAR
        cloud.header.frame_id = "laser"

        cloud.height = 1
        cloud.width = len(self.current_sweep_points)

        cloud.fields = fields

        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.row_step = cloud.point_step * cloud.width
        cloud.data = bytes(data)
        cloud.is_dense = True

        self.map_pub.publish(cloud)

        self.get_logger().info(
            f'Published sweep cloud: {cloud.width} points'
        )

        self.current_sweep_points.clear()

    # ==========================================================
    # Shutdown
    # ==========================================================

    def destroy_node(self):
        self.get_logger().info('Mapping3D stopped')
        super().destroy_node()


def main():

    rclpy.init()

    node = Mapping3D()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()