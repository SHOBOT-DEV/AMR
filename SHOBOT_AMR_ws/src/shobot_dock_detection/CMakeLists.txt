cmake_minimum_required(VERSION 3.5)
project(shobot_dock_detection)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  PROGRAMS
    ${PROJECT_SOURCE_DIR}/shobot_dock_detection/dock_detection_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME dock_detection_node
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
