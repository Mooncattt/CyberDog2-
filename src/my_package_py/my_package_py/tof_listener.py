#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from protocol.msg import HeadTofPayload  # 替换为你的包名和消息名

class TofListener(Node):

    def __init__(self):
        super().__init__('tof_listener')
        self.subscription = self.create_subscription(
            HeadTofPayload,  # 订阅的消息类型
            '/mi_desktop_48_b0_2d_5f_b8_a9/head_tof_payload',  # 订阅的话题
            self.listener_callback,
            10)
        self.latest_left_data = []  # 用于存储左侧数据
        self.latest_right_data = []  # 用于存储右侧数据

    def listener_callback(self, msg):
        # 更新数据
        self.latest_left_data = msg.left_head.data
        self.latest_right_data = msg.right_head.data
        #print(type(self.latest_left_data))
        #self.get_logger().info('Received TOF data:')
        
        # 打印左侧和右侧的 TOF 数据
        #self.print_data(self.latest_left_data, 'Left Head')
        #self.print_data(self.latest_right_data, 'Right Head')
        #if self.check_obstacle(self.latest_left_data):print("have obstacle")

    def print_data(self, data, label):
        """
        打印 8x8 矩阵的 TOF 数据
        """
        self.get_logger().info(f'{label} Data:')
        for i in range(8):
            row = data[i*8:(i+1)*8]  # 每 8 个值表示矩阵的一行
            self.get_logger().info(f'  Row {i}: {row}')

    def check_obstacle(self, data, threshold=0.1):
        """
        检查是否有小于阈值的障碍物
        :param data: 8x8 矩阵数据
        :param threshold: 判断是否为障碍物的距离阈值
        :return: 如果存在小于阈值的值则返回 True
        """
        # 遍历矩阵，检查是否有小于阈值的元素
        return any(d < threshold for d in data)

    def get_latest_data(self):
        """
        返回最近一次接收到的 TOF data 数组
        :return: 左侧和右侧的 TOF 数据
        """
        return self.latest_left_data, self.latest_right_data


# def main(args=None):
#     rclpy.init(args=args)
#     node = TofListener()

#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()
