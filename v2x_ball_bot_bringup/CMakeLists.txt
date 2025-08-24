cmake_minimum_required(VERSION 3.8)
project(v2x_ball_bot_bringup)

find_package(ament_cmake REQUIRED)

# launch / config / worlds / (있으면) models 설치
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY worlds DESTINATION share/${PROJECT_NAME})

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/models")
  install(DIRECTORY models DESTINATION share/${PROJECT_NAME})
endif()

# env hook 템플릿 → 실제 .dsv 생성
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/worlds/gz_resource_path.dsv.in
  ${CMAKE_CURRENT_BINARY_DIR}/gz_resource_path.dsv
  @ONLY
)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/gz_resource_path.dsv
  DESTINATION share/${PROJECT_NAME}/ament_cmake_environment_hooks)

ament_package()
