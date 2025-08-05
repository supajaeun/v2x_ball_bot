cmake_minimum_required(VERSION 3.5)
project(v2x_ball_bot_control)

find_package(ament_cmake REQUIRED)

# ball_detector_node 설치
install(PROGRAMS
  v2x_ball_bot_control/ball_detector_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME ball_detector_node
)

# odom_tf_pub 설치
install(PROGRAMS
  v2x_ball_bot_control/odom_tf_pub.py
  DESTINATION lib/${PROJECT_NAME}
  PERMISSIONS OWNER_EXECUTE OWNER_READ GROUP_EXECUTE GROUP_READ WORLD_EXECUTE WORLD_READ
)

# ✅ pt 파일도 함께 설치
install(FILES
  v2x_ball_bot_control/ball_detector_v3.pt
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
