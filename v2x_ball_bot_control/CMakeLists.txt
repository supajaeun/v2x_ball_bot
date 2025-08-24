cmake_minimum_required(VERSION 3.5)
project(v2x_ball_bot_control)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Python 패키지 설치 (import 편의성)
ament_python_install_package(v2x_ball_bot_control)

# =========================
# Nodes 설치
# =========================

install(PROGRAMS
  v2x_ball_bot_control/motor_driver_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME motor_driver_node
  PERMISSIONS OWNER_EXECUTE OWNER_READ GROUP_EXECUTE GROUP_READ WORLD_EXECUTE WORLD_READ
)


install(PROGRAMS
  v2x_ball_bot_control/visual_servoing_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME visual_servoing_node
  PERMISSIONS OWNER_EXECUTE OWNER_READ GROUP_EXECUTE GROUP_READ WORLD_EXECUTE WORLD_READ
)

# ball_perception_node 설치 (기존 ball_detector_node 대체)
install(PROGRAMS
  v2x_ball_bot_control/ball_perception_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME ball_perception_node
)

# odom_tf_pub 설치
install(PROGRAMS
  v2x_ball_bot_control/odom_tf_pub.py
  DESTINATION lib/${PROJECT_NAME}
  PERMISSIONS OWNER_EXECUTE OWNER_READ GROUP_EXECUTE GROUP_READ WORLD_EXECUTE WORLD_READ
)

# ✅ YOLO 모델 파일 함께 설치
install(FILES
  v2x_ball_bot_control/ball_detector_v3.pt
  DESTINATION lib/${PROJECT_NAME}
)

# ✅ scan_manager 설치
install(PROGRAMS
  v2x_ball_bot_control/scan_manager.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME scan_manager
  PERMISSIONS OWNER_EXECUTE OWNER_READ GROUP_EXECUTE GROUP_READ WORLD_EXECUTE WORLD_READ
)


install(FILES
  config/twist_mux.yaml
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# (선택) 파라미터 YAML share에 설치
install(FILES
  config/scan_manager.yaml
  DESTINATION share/${PROJECT_NAME}/config
)

install(PROGRAMS
  v2x_ball_bot_control/visual_servoing_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME visual_servoing_node
  PERMISSIONS OWNER_EXECUTE OWNER_READ GROUP_EXECUTE GROUP_READ WORLD_EXECUTE WORLD_READ
)

ament_package()
