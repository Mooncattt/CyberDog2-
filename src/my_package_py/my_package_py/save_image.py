import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
import cv2
import os
from datetime import datetime
import time

class RawImageSaver(Node):
    def __init__(self):
        super().__init__('raw_image_saver')
        topic_name = "/image_rgb"
        self.subscription = self.create_subscription(
            Image,
            topic_name,
            self.image_callback,
            10)
        self.output_dir = '/SDCARD/my_ws/src/my_package_py/images'
        os.makedirs(self.output_dir, exist_ok=True)
        self.get_logger().info('RawImageSaver node started, waiting for {}'.format(topic_name))


    def image_callback(self, msg: Image):
        try:
            height = msg.height
            width = msg.width

            if msg.encoding in ['bgr8', 'rgb8']:
                channels = 3
            elif msg.encoding == 'mono8':
                channels = 1
            else:
                self.get_logger().warn(f'Unsupported encoding: {msg.encoding}')
                return

            img_array = np.frombuffer(msg.data, dtype=np.uint8)

            try:
                img_array = img_array.reshape((height, width, channels))
            except ValueError:
                self.get_logger().warn(f'Unexpected step size: {msg.step}, expected: {width * channels}')
                return

        # 如果是 RGB 图像，转换为 OpenCV 所需的 BGR 格式
            if msg.encoding == 'rgb8':
                img_array = cv2.cvtColor(img_array, cv2.COLOR_RGB2BGR)

            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
            filename = os.path.join(self.output_dir, f'image_{timestamp}.jpg')
            cv2.imwrite(filename, img_array)
            self.get_logger().info(f'Saved image to {filename}')
            time.sleep(1)

        except Exception as e:
            self.get_logger().error(f'Error saving image: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = RawImageSaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
