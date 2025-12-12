cmake_minimum_required(VERSION 3.5)
project(shobot_system_monitor)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  PROGRAMS
    ${PROJECT_SOURCE_DIR}/shobot_system_monitor/heartbeat_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME system_heartbeat_node
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
