import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2, Imu
from std_msgs.msg import Header
import numpy as np
import struct
from scipy.spatial.transform import Rotation

class Mapping3D(Node):
    def __init__(self):
        super().__init__('mapping_3d')
        
        # Akumulasi semua titik
        self.accumulated_points = []
        self.latest_imu = None
        
        # Subscriber
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data_raw', self.imu_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        
        # Publisher - map yang terakumulasi
        self.map_pub = self.create_publisher(
            PointCloud2, '/map_3d', 10)
        
        # Publish map setiap 1 detik
        self.timer = self.create_timer(1.0, self.publish_map)
        
        self.get_logger().info('Mapping3D node started!')

    def imu_callback(self, msg):
        self.latest_imu = msg

    def scan_callback(self, msg):
        if self.latest_imu is None:
            self.get_logger().warn('Waiting for IMU data...')
            return

        # Ambil orientasi dari IMU
        q = self.latest_imu.orientation
        rotation = Rotation.from_quat([q.x, q.y, q.z, q.w])
        
        # Konversi scan ke titik 3D dan akumulasi
        angle = msg.angle_min
        for r in msg.ranges:
            if msg.range_min < r < msg.range_max:
                x_local = r * np.cos(angle)
                y_local = r * np.sin(angle)
                z_local = 0.0
                
                point_local = np.array([x_local, y_local, z_local])
                point_global = rotation.apply(point_local)
                self.accumulated_points.append(point_global)
            angle += msg.angle_increment

        # Batasi jumlah titik agar tidak terlalu berat
        max_points = 100000
        if len(self.accumulated_points) > max_points:
            self.accumulated_points = self.accumulated_points[-max_points:]

    def publish_map(self):
        if not self.accumulated_points:
            return
        
        from sensor_msgs.msg import PointField
        
        fields = [
            PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
        ]
        
        data = bytearray()
        for p in self.accumulated_points:
            data += struct.pack('fff', float(p[0]), float(p[1]), float(p[2]))
        
        msg = PointCloud2()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.height = 1
        msg.width = len(self.accumulated_points)
        msg.fields = fields
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = 12 * len(self.accumulated_points)
        msg.data = bytes(data)
        msg.is_dense = True
        
        self.map_pub.publish(msg)
        self.get_logger().info(f'Map published: {len(self.accumulated_points)} total points')

def main():
    rclpy.init()
    node = Mapping3D()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
