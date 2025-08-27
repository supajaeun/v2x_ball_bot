
## WS 전체 클린 빌드
cd ~/ros2_ws
rm -rf build/ install/ log/
colcon build --symlink-install

## bringup, control만 클린 빌드
cd ~/ros2_ws
rm -rf build/v2x_ball_bot_control install/v2x_ball_bot_control log/v2x_ball_bot_control
rm -rf build/v2x_ball_bot_bringup install/v2x_ball_bot_bringup log/v2x_ball_bot_bringup
colcon build --symlink-install --packages-select v2x_ball_bot_control v2x_ball_bot_bringup

## Rviz2 실행
xhost +local:docker
docker exec -it ros2_humble_dev_usb bash
export DISPLAY=:0
rviz2 실행

## depth camera 실행 
ros2 run orbbec_camera orbbec_camera_node \
  --ros-args \
  -p enable_color:=true \
  -p enable_depth:=true \
  -p enable_align_depth_to_color:=true \
  -p color_width:=640 \
  -p color_height:=480 \
  -p color_fps:=30 \
  -p depth_width:=640 \
  -p depth_height:=480 \
  -p depth_fps:=30 \
  -p depth_format:=Y12


## detector node 실행
colcon build --packages-select v2x_ball_bot_control
source install/setup.bash

## 드라이브 링크
https://drive.google.com/drive/folders/1AKHB_Y0bQoje9KC5DslliCRysFL5NmXu?usp=sharing

## map 토픽 발행
docker exec -it main_container bash -lc '
source /opt/ros/humble/setup.bash && source /root/ros2_ws/install/setup.bash &&
ros2 launch v2x_ball_bot_bringup bringup_min.launch.py \
  lidar_port:=/dev/ttyUSB0 lidar_baud:=115200 lidar_scan_mode:=Standard'

## 모터 드라이버 노드 단독 실행
ros2 run v2x_ball_bot_control motor_driver_node \
  --ros-args -p cmd_topic:=/cmd_vel_out \
             -p wheel_radius:=0.05 -p wheel_base:=0.30 \
             -p backend:=topic \
             -p left_topic:=/left_wheel/vel -p right_topic:=/right_wheel/vel
# 시리얼로 MCU에 보내고 싶다면
ros2 run v2x_ball_bot_control motor_driver_node \
  --ros-args -p backend:=serial \
             -p serial_port:=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0 \
             -p serial_baud:=115200

