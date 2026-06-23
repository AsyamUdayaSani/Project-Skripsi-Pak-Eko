import rclpy
from rclpy.node import Node

from sensor_msgs.msg import (
    LaserScan,
    PointCloud2,
    PointField,
    Imu
)

from std_msgs.msg import (
    Header,
    Float32
)

import numpy as np
import struct

from scipy.spatial.transform import Rotation


class Mapping3D(Node):

    def __init__(self):

        super().__init__('mapping_3d')

        self.accumulated_points = []

        self.servo_buffer = []

        self.latest_imu = None

        self.servo_sub = self.create_subscription(
            Float32,
            '/stepper/angle',
            self.servo_callback,
            50
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data_raw',
            self.imu_callback,
            50
        )

        self.map_pub = self.create_publisher(
            PointCloud2,
            '/map_3d',
            10
        )

        self.timer = self.create_timer(
            1.0,
            self.publish_map
        )

        self.get_logger().info(
            'Mapping3D V2 started (LiDAR + Stepper + IMU)'
        )

    def imu_callback(self, msg):

        self.latest_imu = msg

    def get_time_sec(self, header=None, clock_time=None):

        if header:
            return (
                header.stamp.sec
                +
                header.stamp.nanosec * 1e-9
            )

        if clock_time:
            return (
                clock_time.nanoseconds * 1e-9
            )

    def servo_callback(self, msg):

        t = self.get_time_sec(
            clock_time=self.get_clock().now()
        )

        angle_rad = msg.data

        self.servo_buffer.append(
            (
                t,
                angle_rad
            )
        )

        if len(self.servo_buffer) > 100:
            self.servo_buffer.pop(0)

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

        times = np.array([
            item[0]
            for item in self.servo_buffer
        ])

        angles = np.array([
            item[1]
            for item in self.servo_buffer
        ])

        t_start = self.get_time_sec(
            header=msg.header
        )

        time_increment = msg.time_increment

        if time_increment <= 0.0:

            time_increment = (
                (1.0 / 10.0)
                /
                len(msg.ranges)
            )

        scan_angle = msg.angle_min

        for i, r in enumerate(msg.ranges):

            if msg.range_min < r < msg.range_max:

                t_ray = (
                    t_start
                    +
                    (i * time_increment)
                )

                interpolated_servo_angle = np.interp(
                    t_ray,
                    times,
                    angles
                )  

                x_local = (
                    r *
                    np.cos(scan_angle)
                )

                y_local = (
                    r *
                    np.sin(scan_angle)
                )

                z_local = 0.0

                point_local = np.array([
                    x_local,
                    y_local,
                    z_local
                ])

                stepper_rotation = (
                    Rotation.from_euler(
                        'x',
                        -interpolated_servo_angle
                    )
                )

                combined_rotation = (
                    imu_rotation
                    *
                    stepper_rotation
                )

                point_global = (
                    combined_rotation.apply(
                        point_local
                    )
                )

                self.accumulated_points.append(
                    point_global
                )

            scan_angle += msg.angle_increment

        max_points = 100000

        if len(self.accumulated_points) > max_points:

            self.accumulated_points = (
                self.accumulated_points[
                    -max_points:
                ]
            )

    def publish_map(self):

        if not self.accumulated_points:
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

        for p in self.accumulated_points:

            data += struct.pack(
                'fff',
                float(p[0]),
                float(p[1]),
                float(p[2])
            )

        msg = PointCloud2()

        msg.header = Header()

        msg.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        msg.header.frame_id = 'base_link'

        msg.height = 1

        msg.width = len(
            self.accumulated_points
        )

        msg.fields = fields

        msg.is_bigendian = False

        msg.point_step = 12

        msg.row_step = (
            12 *
            len(
                self.accumulated_points
            )
        )

        msg.data = bytes(data)

        msg.is_dense = True

        self.map_pub.publish(msg)

    def destroy_node(self):

        self.get_logger().info(
            'Mapping3D stopped'
        )

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