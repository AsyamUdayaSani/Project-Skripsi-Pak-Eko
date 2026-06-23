import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_msgs.msg import Header, Float32
import numpy as np
import struct
from scipy.spatial.transform import Rotation

class ScanToPointCloud3D(Node):
    def __init__(self):
        super().__init__('scan_to_pointcloud3d')
        
        self.servo_buffer = []
        
        self.servo_sub = self.create_subscription(
            Float32, '/stepper/angle', self.servo_callback, 50)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        
        self.pub = self.create_publisher(
            PointCloud2, '/point_cloud_3d', 10)
        
        self.get_logger().info('ScanToPointCloud3D node started!')

    def get_time_sec(self, header=None, clock_time=None):
        if header:
            return header.stamp.sec + header.stamp.nanosec * 1e-9
        if clock_time:
            return clock_time.nanoseconds * 1e-9

    def servo_callback(self, msg):
        t = self.get_time_sec(clock_time=self.get_clock().now())
        self.servo_buffer.append((t, msg.data))
        if len(self.servo_buffer) > 100:
            self.servo_buffer.pop(0)

    def scan_callback(self, msg):
        if len(self.servo_buffer) < 2:
            return

        times = np.array([item[0] for item in self.servo_buffer])
        angles = np.array([item[1] for item in self.servo_buffer])

        t_start = self.get_time_sec(header=msg.header)
        time_increment = msg.time_increment if msg.time_increment > 0 else (1.0 / 10.0) / len(msg.ranges)
        
        points = []
        scan_angle = msg.angle_min
        
        for i, r in enumerate(msg.ranges):
            if msg.range_min < r < msg.range_max:
                # Same filter for the actuator shafts
                if (1.4 < scan_angle < 1.7) or (-1.7 < scan_angle < -1.4):
                    scan_angle += msg.angle_increment
                    continue

                t_ray = t_start + (i * time_increment)
                interpolated_angle = np.interp(t_ray, times, angles)

                x_local = r * np.cos(scan_angle)
                y_local = r * np.sin(scan_angle)
                
                point_local = np.array([x_local, y_local, 0.0])
                rotation = Rotation.from_euler('y', interpolated_angle)
                
                points.append(rotation.apply(point_local))
                
            scan_angle += msg.angle_increment

        cloud_msg = self.create_pointcloud2(points, msg.header)
        self.pub.publish(cloud_msg)

    def create_pointcloud2(self, points, header):
        fields = [
            PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
        ]
        
        data = bytearray()
        for p in points:
            data += struct.pack('fff', float(p[0]), float(p[1]), float(p[2]))
        
        msg = PointCloud2()
        msg.header = header
        msg.header.frame_id = 'base_link'
        msg.height = 1
        msg.width = len(points)
        msg.fields = fields
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = 12 * len(points)
        msg.data = bytes(data)
        msg.is_dense = True
        
        return msg

def main():
    rclpy.init()
    node = ScanToPointCloud3D()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()