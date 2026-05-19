import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
import lgpio
import time

# Pin GPIO
STEP_PIN = 17
DIR_PIN  = 27
EN_PIN   = 22

class StepperNode(Node):
    def __init__(self):
        super().__init__('stepper_node')

        # Setup GPIO
        self.h = lgpio.gpiochip_open(4)
        lgpio.gpio_claim_output(self.h, STEP_PIN)
        lgpio.gpio_claim_output(self.h, DIR_PIN)
        lgpio.gpio_claim_output(self.h, EN_PIN)

        # Default: motor aktif (EN = LOW untuk TMC2209)
        lgpio.gpio_write(self.h, EN_PIN, 0)
        lgpio.gpio_write(self.h, DIR_PIN, 1)

        # Parameter
        self.declare_parameter('steps_per_rev', 200)  # 200 step = 1 putaran full
        self.declare_parameter('delay', 0.001)         # kecepatan (detik per step)
        self.declare_parameter('direction', 1)         # 1 = CW, 0 = CCW

        self.steps_per_rev = self.get_parameter('steps_per_rev').value
        self.delay         = self.get_parameter('delay').value
        self.direction     = self.get_parameter('direction').value

        # Subscriber — kontrol arah dan kecepatan
        self.create_subscription(Bool,    '/stepper/enable',    self.enable_cb,    10)
        self.create_subscription(Bool,    '/stepper/direction', self.direction_cb, 10)
        self.create_subscription(Float32, '/stepper/speed',     self.speed_cb,     10)

        # Timer — putar motor terus menerus untuk scanning
        self.running = True
        self.timer = self.create_timer(self.delay, self.step_once)

        self.get_logger().info('Stepper node started!')

    def step_once(self):
        if not self.running:
            return
        lgpio.gpio_write(self.h, STEP_PIN, 1)
        time.sleep(self.delay / 2)
        lgpio.gpio_write(self.h, STEP_PIN, 0)
        time.sleep(self.delay / 2)

    def enable_cb(self, msg):
        self.running = msg.data
        # EN pin: LOW = aktif, HIGH = nonaktif
        lgpio.gpio_write(self.h, EN_PIN, 0 if msg.data else 1)
        self.get_logger().info(f'Motor {"enabled" if msg.data else "disabled"}')

    def direction_cb(self, msg):
        self.direction = 1 if msg.data else 0
        lgpio.gpio_write(self.h, DIR_PIN, self.direction)
        self.get_logger().info(f'Direction: {"CW" if msg.data else "CCW"}')

    def speed_cb(self, msg):
        # msg.data = delay dalam detik (misal 0.001 = cepat, 0.005 = lambat)
        self.delay = float(msg.data)
        self.get_logger().info(f'Speed delay set to: {self.delay}s')

    def destroy_node(self):
        lgpio.gpio_write(self.h, EN_PIN, 1)  # Disable motor saat shutdown
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
