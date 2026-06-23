import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Int32
from geometry_msgs.msg import TransformStamped
import tf2_ros
import lgpio
import time
import math
import threading

STEP_PIN = 17
DIR_PIN  = 27
EN_PIN   = 22
GPIOCHIP = 4  # Raspberry Pi 5

class StepperNode(Node):
    def __init__(self):
        super().__init__('stepper_node')

        self.h = lgpio.gpiochip_open(GPIOCHIP)
        lgpio.gpio_claim_output(self.h, STEP_PIN)
        lgpio.gpio_claim_output(self.h, DIR_PIN)
        lgpio.gpio_claim_output(self.h, EN_PIN)
        lgpio.gpio_write(self.h, EN_PIN, 0)
        lgpio.gpio_write(self.h, DIR_PIN, 1)

        self.declare_parameter('steps_per_rev', 1600)
        self.declare_parameter('delay', 0.005)
        self.declare_parameter('direction', 1)

        self.steps_per_rev = self.get_parameter('steps_per_rev').value
        self.delay         = self.get_parameter('delay').value
        self.direction     = self.get_parameter('direction').value

        self.current_step = 0
        self.running      = True
        
        # --- GEAR RATIO CONFIGURATION ---
        # Motor Pulley (30T) / LiDAR Pulley (60T) = 0.5
        # This means the LiDAR rotates at exactly half the speed of the motor.
        self.gear_ratio = 30.0 / 60.0 

        self.angle_pub  = self.create_publisher(Float32, '/stepper/angle',  10)
        self.steps_pub  = self.create_publisher(Int32,   '/stepper/steps',  10)
        self.status_pub = self.create_publisher(Bool,    '/stepper/status', 10)

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.create_subscription(Bool,    '/stepper/enable',    self.enable_cb,    10)
        self.create_subscription(Bool,    '/stepper/direction', self.direction_cb, 10)
        self.create_subscription(Float32, '/stepper/speed',     self.speed_cb,     10)

        # Background thread to prevent ROS executor blocking
        self.motor_thread = threading.Thread(target=self.motor_loop)
        self.motor_thread.daemon = True
        self.motor_thread.start()

        self.get_logger().info('Stepper node started with 1:2 gear ratio math!')

    def motor_loop(self):
        while rclpy.ok():
            if not self.running:
                time.sleep(0.1)
                continue

            lgpio.gpio_write(self.h, STEP_PIN, 1)
            time.sleep(self.delay / 2.0)
            lgpio.gpio_write(self.h, STEP_PIN, 0)
            time.sleep(self.delay / 2.0)

            if self.direction == 1:
                self.current_step += 1
            else:
                self.current_step -= 1

            # --- NEW MATH FOR GEAR RATIO ---
            # 1. Calculate the total continuous degrees the motor has turned
            motor_angle_deg = (self.current_step / self.steps_per_rev) * 360.0
            
            # 2. Apply gear ratio to find the actual physical angle of the Lidar.
            # Modulo 360 ensures the resulting Lidar angle stays within a standard circle.
            # It now correctly takes 400 motor steps to complete one 360-degree Lidar rotation.
            lidar_angle_deg = (motor_angle_deg * self.gear_ratio) % 360.0
            
            # 3. Convert to Radians for 3D math and TF standard compliance
            lidar_angle_rad = math.radians(lidar_angle_deg)

            stamp = self.get_clock().now().to_msg()

            # Publish the TRUE Lidar angle so mapping nodes don't have to do extra math
            angle_msg = Float32()
            angle_msg.data = lidar_angle_rad
            self.angle_pub.publish(angle_msg)

            steps_msg = Int32()
            steps_msg.data = self.current_step
            self.steps_pub.publish(steps_msg)

            status_msg = Bool()
            status_msg.data = self.running
            self.status_pub.publish(status_msg)

            self.broadcast_tf(lidar_angle_rad, stamp)

    def broadcast_tf(self, angle_rad, stamp):
        t = TransformStamped()
        t.header.stamp    = stamp
        t.header.frame_id = 'base_link'
        t.child_frame_id  = 'lidar_tilt'

        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.1

        t.transform.rotation.x = math.sin(angle_rad / 2)
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = math.cos(angle_rad / 2)

        self.tf_broadcaster.sendTransform(t)

    def enable_cb(self, msg):
        self.running = msg.data
        lgpio.gpio_write(self.h, EN_PIN, 0 if msg.data else 1)

    def direction_cb(self, msg):
        self.direction = 1 if msg.data else 0
        lgpio.gpio_write(self.h, DIR_PIN, self.direction)

    def speed_cb(self, msg):
        self.delay = float(msg.data)

    def destroy_node(self):
        lgpio.gpio_write(self.h, EN_PIN, 1)
        lgpio.gpiochip_close(self.h)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = StepperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()