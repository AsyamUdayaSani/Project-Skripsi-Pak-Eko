import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Int32
from geometry_msgs.msg import TransformStamped
import tf2_ros
import lgpio
import time
import math

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

        self.declare_parameter('steps_per_rev', 200)
        self.declare_parameter('delay', 0.005)
        self.declare_parameter('direction', 1)

        self.steps_per_rev = self.get_parameter('steps_per_rev').value
        self.delay         = self.get_parameter('delay').value
        self.direction     = self.get_parameter('direction').value

        self.current_step = 0
        self.running      = True

        self.angle_pub  = self.create_publisher(Float32, '/stepper/angle',  10)
        self.steps_pub  = self.create_publisher(Int32,   '/stepper/steps',  10)
        self.status_pub = self.create_publisher(Bool,    '/stepper/status', 10)

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.create_subscription(Bool,    '/stepper/enable',    self.enable_cb,    10)
        self.create_subscription(Bool,    '/stepper/direction', self.direction_cb, 10)
        self.create_subscription(Float32, '/stepper/speed',     self.speed_cb,     10)

        self.timer = self.create_timer(self.delay, self.step_once)

        self.get_logger().info('Stepper node started!')

    def step_once(self):
        if not self.running:
            return

        lgpio.gpio_write(self.h, STEP_PIN, 1)
        time.sleep(self.delay / 2)
        lgpio.gpio_write(self.h, STEP_PIN, 0)
        time.sleep(self.delay / 2)

        if self.direction == 1:
            self.current_step += 1
        else:
            self.current_step -= 1

        angle_deg = (self.current_step % self.steps_per_rev) \
                    / self.steps_per_rev * 360.0
        angle_rad = math.radians(angle_deg)

        angle_msg = Float32()
        angle_msg.data = angle_deg
        self.angle_pub.publish(angle_msg)

        steps_msg = Int32()
        steps_msg.data = self.current_step
        self.steps_pub.publish(steps_msg)

        status_msg = Bool()
        status_msg.data = self.running
        self.status_pub.publish(status_msg)

        self.broadcast_tf(angle_rad)

    def broadcast_tf(self, angle_rad):
        t = TransformStamped()
        t.header.stamp    = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id  = 'lidar_tilt'

        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.1

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = math.sin(angle_rad / 2)
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = math.cos(angle_rad / 2)

        self.tf_broadcaster.sendTransform(t)

    def enable_cb(self, msg):
        self.running = msg.data
        lgpio.gpio_write(self.h, EN_PIN, 0 if msg.data else 1)
        self.get_logger().info(f'Motor {"enabled" if msg.data else "disabled"}')

    def direction_cb(self, msg):
        self.direction = 1 if msg.data else 0
        lgpio.gpio_write(self.h, DIR_PIN, self.direction)
        self.get_logger().info(f'Direction: {"CW" if msg.data else "CCW"}')

    def speed_cb(self, msg):
        self.delay = float(msg.data)
        self.get_logger().info(f'Speed delay: {self.delay}s')

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
