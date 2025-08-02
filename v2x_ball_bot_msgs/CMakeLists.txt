cmake_minimum_required(VERSION 3.8)
project(v2x_ball_bot_msgs)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 메시지 파일 추가
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BallPixel.msg"
  "msg/BallPosition.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
