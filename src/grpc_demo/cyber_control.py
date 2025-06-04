# -*- coding:utf-8 -*-

import sys
sys.path.append("/SDCARD/my_ws/src/grpc_demo")
sys.path.append("/SDCARD/my_ws/src/my_package_py/my_package_py")

import os
import rclpy
from datetime import datetime
import threading
import cv2
from red_ball_detect import RedBalloonDetector, RawImageSaver
from grpc_teleop import Client, Teleop, ProtoEncoder
import numpy as np

class BalloonFollower:
    def __init__(self, cyberdog_ip, ca_cert, client_key, client_cert,ros_node):
        self.detector = RedBalloonDetector(min_circularity= 0.3)
        self.client = Client(cyberdog_ip, ca_cert, client_key, client_cert)
        self.teleop = Teleop(acc=[0.5, 0.0, 3.0], freq=30.0, max_vel=[0.6, 0.0, 1.0])
        self.encoder = ProtoEncoder()
        self.area_threshold = 0.1  # Adjust based on balloon size
        self.center_x_range = (0.45, 0.55)
        self.ros_node = ros_node


    def process_image(self, image):
        result = self.detector.detect(image)
        current_range = self.ros_node.get_latest_range()
        left_tof, right_tof = self.ros_node.get_tof_data()
        if not result or not current_range or not left_tof or not right_tof:
            self.send_stop()
            return
        x_ratio, y_ratio, area_ratio = result
        cx = int(image.shape[1] * x_ratio)
        cy = int(image.shape[0] * y_ratio)
        cv2.circle(image, (cx, cy), 3, (0, 255, 0), -1)
        print(f"红气球中心: x={x_ratio:.3f}, y={y_ratio:.3f}, area={area_ratio:.3f}")

        print(f"area_ratio: {area_ratio}")

        
        print(f"当前超声波距离: {current_range:.2f} 米")

        
        print("TOF 左侧中心值：", np.mean(left_tof), "右侧中心值：", np.mean(right_tof))

        # if self.ros_node.check_obstacle(left_tof) or self.ros_node.check_obstacle(right_tof) or current_range < 0.22:
        #     self.send_stop()
        #     print("Obstacle!!")
        #     return


        if area_ratio >= self.area_threshold:
            self.send_stop()
            print("Arrive!!")
            return

        # ----------- method 1
        # delta_vel = [0.0, 0.0, 0.0]
        # if x_ratio < self.center_x_range[0]:
        #     delta_vel[2] = self.teleop._Teleop__acc[2] / self.teleop._Teleop__freq * (0.5 - x_ratio) * 2 # Turn left
        #     # self.teleop.set_linear_vel(0)
        # elif x_ratio > self.center_x_range[1]:
        #     delta_vel[2] = -self.teleop._Teleop__acc[2] / self.teleop._Teleop__freq * (x_ratio - 0.5) * 2 # Turn right
        #     # self.teleop.set_linear_vel(0)
        # else:
        #     self.teleop.set_angle_vel(0)
        #     delta_vel[0] = self.teleop._Teleop__acc[0] / self.teleop._Teleop__freq  # Move forward
        # print("vel={}, delta_vel={}".format(vel, delta_vel))

        # ----------- method 2
        vel = [0.0, 0.0, 0.0]
        if x_ratio < self.center_x_range[0]:
            vel[2] = (0.5 - x_ratio) * 2 # Turn left
        elif x_ratio > self.center_x_range[1]:
            vel[2] = -(x_ratio - 0.5) * 2 # Turn right
        if self.Check(left_tof,right_tof,current_range): 
            area_ratio = self.area_threshold
        vel[0] = (self.area_threshold - area_ratio) * 5
        
        print("vel={}".format(vel))

        json_str = self.encoder.encodeVel(vel)
        self.client.sendMsg(1002, json_str)

    def send_stop(self):
        json_str = self.encoder.stopSignal()
        self.client.sendMsg(1002, json_str)
    
    def Check(self,left_tof,right_tof,current_range):
        return self.ros_node.check_obstacle(left_tof) or self.ros_node.check_obstacle(right_tof) or current_range < 0.22

def rosspin(node):
    rclpy.spin(node)


def rosspin_thread(node):
    ros_thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
    ros_thread.start()


def main(cyberdog_ip, ca_cert, client_key, client_cert):
    rclpy.init(args=None)
    img_node = RawImageSaver(Det=False)

    follower = BalloonFollower(cyberdog_ip, ca_cert, client_key, client_cert,img_node)
    
    #rclpy.init(args=None)
    

    rosspin_thread(img_node)

    try:
        while True:
            
            if not os.path.exists("/SDCARD/my_ws/src/my_package_py/get_img/cam.jpg"):
                continue
            frame = cv2.imread("/SDCARD/my_ws/src/my_package_py/get_img/cam.jpg")
            follower.process_image(frame)

            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
            filename = os.path.join('/SDCARD/my_ws/src/my_package_py/images', f'image_{timestamp}.jpg')

            cv2.imwrite(filename, frame)
            if 0xFF == ord('q'):
                break


    finally:
        img_node.destroy_node()
        rclpy.shutdown()
        
        follower.send_stop()



if __name__ == "__main__":

    if len(sys.argv) != 5:
        print("Usage: python balloon_follower.py <cyberdog_ip> <ca_cert> <client_key> <client_cert>")
        sys.exit(1)
    main(sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4])