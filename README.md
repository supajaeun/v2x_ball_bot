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