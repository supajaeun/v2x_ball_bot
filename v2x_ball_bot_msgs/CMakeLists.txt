cmake_minimum_required(VERSION 3.8)
project(v2x_ball_bot_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# 실제 보유 msg 파일만 나열
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BallPixel.msg"
  "msg/BallPosition.msg"
  "msg/Ball.msg"
  "msg/BallArray.msg"
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
