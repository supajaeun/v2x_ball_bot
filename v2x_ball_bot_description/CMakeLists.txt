cmake_minimum_required(VERSION 3.8)
project(v2x_ball_bot_description)

find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)

# 있는 디렉터리만 설치 (각 폴더 존재 여부 검사)
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/urdf)
  install(DIRECTORY urdf/ DESTINATION share/${PROJECT_NAME}/urdf)
endif()

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/meshes)
  install(DIRECTORY meshes gz DESTINATION share/${PROJECT_NAME})
endif()

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/launch)
  install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
endif()

ament_package()
