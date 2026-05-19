import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2, Imu
import numpy as np
import struct
from scipy.spatial.transform import Rotation

class ScanToPointCloud3D(Node):
    def __init__(self):
        super().__init__('scan_to_pointcloud3d')
        
        # Simpan data IMU terbaru
        self.latest_imu = None
        
        # Subscriber
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data_raw', self.imu_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        
        # Publisher
        self.pub = self.create_publisher(
            PointCloud2, '/point_cloud_3d', 10)
        
        self.get_logger().info('ScanToPointCloud3D node started!')

    def imu_callback(self, msg):
        self.latest_imu = msg

    def scan_callback(self, msg):
        if self.latest_imu is None:
            self.get_logger().warn('Waiting for IMU data...')
            return

        # Ambil orientasi dari IMU (quaternion)
        q = self.latest_imu.orientation
        rotation = Rotation.from_quat([q.x, q.y, q.z, q.w])
        
        # Konversi LaserScan ke titik-titik 3D
        points = []
        angle = msg.angle_min
        for r in msg.ranges:
            if msg.range_min < r < msg.range_max:
                # Koordinat lokal sensor (2D)
                x_local = r * np.cos(angle)
                y_local = r * np.sin(angle)
                z_local = 0.0
                
                # Rotasi menggunakan orientasi IMU
                point_local = np.array([x_local, y_local, z_local])
                point_global = rotation.apply(point_local)
                
                points.append(point_global)
            angle += msg.angle_increment

        # Buat pesan PointCloud2
        cloud_msg = self.create_pointcloud2(points, msg.header)
        self.pub.publish(cloud_msg)
        self.get_logger().info(f'Published {len(points)} points!')

    def create_pointcloud2(self, points, header):
        from sensor_msgs.msg import PointField
        
        fields = [
            PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
        ]
        
        point_step = 12  # 3 x float32 = 12 bytes
        data = bytearray()
        for p in points:
            data += struct.pack('fff', float(p[0]), float(p[1]), float(p[2]))
        
        msg = PointCloud2()
        msg.header = header
        msg.header.frame_id = 'imu_link'
        msg.height = 1
        msg.width = len(points)
        msg.fields = fields
        msg.is_bigendian = False
        msg.point_step = point_step
        msg.row_step = point_step * len(points)
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
