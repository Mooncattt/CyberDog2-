# CyberDog2-
CyberDog2仿生机器狗目标跟踪应用研究

该项目可使 CyberDog2 实时跟踪红球

将该 “src ”文档放入 /SDCARD/my_ws 中


run dog by teleop command:

cd /SDCARD/my_ws/src/grpc_demo 

python3 grpc_teleop.py 192.168.43.23 cert/ca-cert.pem cert/client-key.pem cert/client-cert.pem


spawn camera:

ros2 launch realsense2_camera on_dog.py 

ros2 lifecycle set /camera/camera configure 

ros2 lifecycle set /camera/camera activate 

ros2 launch camera_test stereo_camera.py 

ros2 lifecycle set /stereo_camera configure 

ros2 lifecycle set /stereo_camera activate


save image:

python3 /SDCARD/my_ws/src/my_package_py/my_package_py/save_image.py


detect red ball in image:

python3 /SDCARD/my_ws/src/my_package_py/my_package_py/red_ball_detect.py

test the supersonic:

cd /SDCARD/my_ws/src/

ros2 run my_package_py my_package_py ultrasonic_listener

test the tof:

cd /SDCARD/my_ws/src/

ros2 run my_package_py my_package_py tof_listener

cyber control to trace red ball:

cd /SDCARD/my_ws/src/grpc_demo/ 

python3 cyber_control.py 192.168.43.23 cert/ca-cert.pem cert/client-key.pem cert/client-cert.pem
