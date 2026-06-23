import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from laser_geometry import LaserProjection
import tf2_ros

class ScanToPointCloud(Node):
    def __init__(self):
        super().__init__('scan_to_pointcloud')
        self.projector = LaserProjection()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.pub = self.create_publisher(
            PointCloud2, '/point_cloud', 10)

    def scan_callback(self, msg):
        try:
            cloud = self.projector.projectLaser(msg)
            self.pub.publish(cloud)
            self.get_logger().info('PointCloud2 published!')
        except Exception as e:
            self.get_logger().error(f'Error: {str(e)}')

def main():
    rclpy.init()
    node = ScanToPointCloud()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
