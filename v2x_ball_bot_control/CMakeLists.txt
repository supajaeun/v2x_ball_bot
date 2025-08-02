cmake_minimum_required(VERSION 3.5)
project(v2x_ball_bot_control)

find_package(ament_cmake REQUIRED)

# ball_detector_node.py만 install + RENAME
install(PROGRAMS
  v2x_ball_bot_control/ball_detector_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME ball_detector_node
)

# trajectory_predictor_node.py도 따로 install + RENAME
install(PROGRAMS
  v2x_ball_bot_control/trajectory_predictor_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME trajectory_predictor_node
)

ament_package()
