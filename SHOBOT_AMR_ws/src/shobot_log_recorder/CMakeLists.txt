cmake_minimum_required(VERSION 3.5)
project(shobot_log_recorder)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  PROGRAMS
    ${PROJECT_SOURCE_DIR}/shobot_log_recorder/log_recorder_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME log_recorder_node
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
