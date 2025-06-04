import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range

class UltrasonicListener(Node):
    def __init__(self):
        super().__init__('ultrasonic_listener')
        self._latest_range = None  # 用于存储最新的 range 值
        self.subscription = self.create_subscription(
            Range,
            '/mi_desktop_48_b0_2d_5f_b8_a9/ultrasonic_payload',  # 确保订阅正确的 topic
            self.callback,
            10
        )

    def callback(self, msg: Range):
        self._latest_range = msg.range
        self.get_logger().info(f'Received distance: {self._latest_range:.3f} m')

    def get_latest_range(self):
        """返回最新接收到的range值，如果尚未接收到则返回None"""
        return self._latest_range

def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
