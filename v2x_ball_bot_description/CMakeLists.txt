cmake_minimum_required(VERSION 3.8)
project(v2x_ball_bot_description)
find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)
install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)
ament_package() 