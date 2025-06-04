import cv2
import numpy as np
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import Range  
from datetime import datetime
import time
from protocol.msg import HeadTofPayload 


class RawImageSaver(Node):
    def __init__(self, Det=True):
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
        
        if Det:
            self.detector = RedBalloonDetector()
        self.Real_detect = Det

    #ultrasonic
        self._latest_range = None
        self.range_subscription = self.create_subscription(
            Range,
            '/mi_desktop_48_b0_2d_5f_b8_a9/ultrasonic_payload',
            self.range_callback,
            10
        )
        #tof
        self.latest_left_data = None
        self.latest_right_data = None
        self.tof_subscription = self.create_subscription(
            HeadTofPayload,  # 订阅的消息类型
            '/mi_desktop_48_b0_2d_5f_b8_a9/head_tof_payload',  # 订阅的话题
            self.tof_callback,
            10)

    #ultrasonic
    def range_callback(self, msg: Range):
        self._latest_range = msg.range
        #self.get_logger().info(f'Received distance: {self._latest_range:.3f} m')

    def get_latest_range(self):
        return self._latest_range
    

    #tof
    def tof_callback(self,msg:HeadTofPayload):
        self.latest_left_data = msg.left_head.data
        self.latest_right_data = msg.right_head.data
        #self.get_logger().info('Received TOF data')
        #self.print_data(self.latest_left_data, 'Left Head')
        #self.print_data(self.latest_right_data, 'Right Head')

    def get_tof_data(self):
        return self.latest_left_data,self.latest_right_data

    def print_data(self, data, label):
        """
        打印 8x8 矩阵的 TOF 数据
        """
        self.get_logger().info(f'{label} Data:')
        for i in range(8):
            row = data[i*8:(i+1)*8]  # 每 8 个值表示矩阵的一行
            self.get_logger().info(f'  Row {i}: {row}')

    def check_obstacle(self, data, threshold=0.25):
        """
        检查是否有小于阈值的障碍物
        :param data: 8x8 矩阵数据
        :param threshold: 判断是否为障碍物的距离阈值
        :return: 如果存在小于阈值的值则返回 True
        """
        # 遍历矩阵，检查是否有小于阈值的元素
        return any(d < threshold for d in data)
        

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

            if self.Real_detect:
                timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
                filename = os.path.join(self.output_dir, f'image_{timestamp}.jpg')

                result = self.detector.detect(img_array)
                if result:
                    (x_ratio, y_ratio, area_ratio) = result
                    cx = int(img_array.shape[1] * x_ratio)
                    cy = int(img_array.shape[0] * y_ratio)
                    cv2.circle(img_array, (cx, cy), 3, (0, 255, 0), -1)

                    print(f"{filename} 红气球中心: x={x_ratio:.3f}, y={y_ratio:.3f}, area={area_ratio:.3f}")
                else:
                    print(f"{filename} 未检测到红气球")

        
                cv2.imwrite(filename, img_array)
                self.get_logger().info(f'Saved image to {filename}')
                time.sleep(1)

            cv2.imwrite("/SDCARD/my_ws/src/my_package_py/get_img/cam_temp.jpg", img_array)
            os.rename("/SDCARD/my_ws/src/my_package_py/get_img/cam_temp.jpg", "/SDCARD/my_ws/src/my_package_py/get_img/cam.jpg")


        except Exception as e:
            self.get_logger().error(f'Error saving image: {e}')


class RedBalloonDetector:
    def __init__(self, min_circularity= 0.2):
        # HSV 中红色分布在两个区域（低区和高区）
        self.lower_red1 = np.array([0, 100, 100])
        self.upper_red1 = np.array([10, 255, 255])
        self.lower_red2 = np.array([160, 100, 100])
        self.upper_red2 = np.array([179, 255, 255])
        self.min_circularity = min_circularity  # 最小圆形度阈值（可根据需要调整）


    def detect(self, image):
        """
        输入 BGR 图像，返回气球中心点的归一化坐标 (x_ratio, y_ratio, area_ratio)
        如果未检测到或区域不够圆，返回 None
        """
        # 转换为 HSV 颜色空间
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # 生成两个红色掩码，并合并
        mask1 = cv2.inRange(hsv, self.lower_red1, self.upper_red1)
        mask2 = cv2.inRange(hsv, self.lower_red2, self.upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)

        # 对图像进行一些清理
        mask = cv2.medianBlur(mask, 5)

        # 找轮廓
        _, contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            return None  # 没找到气球

        # 选择最大轮廓（假设是气球）
        largest = max(contours, key=cv2.contourArea)
        
        # 计算圆形度
        perimeter = cv2.arcLength(largest, True)
        if perimeter == 0:
            return None
        area = cv2.contourArea(largest)
        circularity = 4 * np.pi * area / (perimeter ** 2)
        
        # 检查圆形度是否足够高
        print("circularity={}".format(circularity))
        if circularity < self.min_circularity:
            return None

        M = cv2.moments(largest)

        if M["m00"] == 0:
            return None

        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])

        contour_area = cv2.contourArea(largest)
        image_area = image.shape[0] * image.shape[1]
        area_ratio = contour_area / image_area

        h, w = image.shape[:2]
        x_ratio = cx / w
        y_ratio = cy / h

        return (x_ratio, y_ratio, area_ratio)



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


if __name__ == "__main__":
    # 检测文件夹下所有图片
    main()
