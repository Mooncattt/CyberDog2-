# -*- coding:utf-8 -*-

import sys
sys.path.append("/SDCARD/my_ws/src/grpc_demo")
sys.path.append("/SDCARD/my_ws/src/my_package_py/my_package_py")

from datetime import datetime
import cv2
from red_ball_detect import RedBalloonDetector
from grpc_teleop import Client, Teleop, ProtoEncoder
from my_package.ultrasonic_listener import UltrasonicListener

class BalloonFollower:
    def __init__(self, cyberdog_ip, ca_cert, client_key, client_cert):
        self.detector = RedBalloonDetector()
        self.client = Client(cyberdog_ip, ca_cert, client_key, client_cert)
        self.teleop = Teleop(acc=[0.5, 0.0, 3.0], freq=10.0, max_vel=[1.0, 0.0, 0.5])
        self.encoder = ProtoEncoder()
        self.area_threshold = 0.08  # Adjust based on balloon size
        self.center_x_range = (0.4, 0.6)
        self.ultrasonic_listener = UltrasonicListener()

    def process_image(self, image):
        result = self.detector.detect(image)
        if not result:
            self.send_stop()
            return
        x_ratio, y_ratio, area_ratio = result
        cx = int(image.shape[1] * x_ratio)
        cy = int(image.shape[0] * y_ratio)
        cv2.circle(image, (cx, cy), 3, (0, 255, 0), -1)
        print(f"红气球中心: x={x_ratio:.3f}, y={y_ratio:.3f}, area={area_ratio:.3f}")

        current_range = self.ultrasonic_listener.get_latest_range()
        print(f"当前超声波距离: {current_range:.2f} 米")

        if area_ratio >= self.area_threshold or current_range<0.21:
            self.send_stop()
            return

        delta_vel = [0.0, 0.0, 0.0]
        if x_ratio < self.center_x_range[0]:
            delta_vel[2] = self.teleop._Teleop__acc[2] / self.teleop._Teleop__freq  # Turn left
        elif x_ratio > self.center_x_range[1]:
            delta_vel[2] = -self.teleop._Teleop__acc[2] / self.teleop._Teleop__freq  # Turn right
        else:
            delta_vel[0] = self.teleop._Teleop__acc[0] / self.teleop._Teleop__freq  # Move forward

        vel = self.teleop.updateVel(delta_vel)
        json_str = self.encoder.encodeVel(vel)
        self.client.sendMsg(1002, json_str)

    def send_stop(self):
        json_str = self.encoder.stopSignal()
        self.client.sendMsg(1002, json_str)

def main(cyberdog_ip, ca_cert, client_key, client_cert):
    follower = BalloonFollower(cyberdog_ip, ca_cert, client_key, client_cert)
    cap = cv2.VideoCapture(0)  # Use camera input
    
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            follower.process_image(frame)

            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
            filename = os.path.join(self.output_dir, f'image_{timestamp}.jpg')

            cv2.imwrite(filename, frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()
        follower.send_stop()

if __name__ == "__main__":
    import sys
    if len(sys.argv) != 5:
        print("Usage: python balloon_follower.py <cyberdog_ip> <ca_cert> <client_key> <client_cert>")
        sys.exit(1)
    main(sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4])