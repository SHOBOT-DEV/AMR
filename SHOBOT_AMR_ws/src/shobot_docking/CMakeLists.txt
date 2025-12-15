cmake_minimum_required(VERSION 3.5)
project(shobot_docking)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Dock.action"
  DEPENDENCIES geometry_msgs std_msgs
)

# Avoid duplicate target errors if this file is processed more than once
if(NOT TARGET ament_cmake_python_symlink_${PROJECT_NAME})
  ament_python_install_package(${PROJECT_NAME})
endif()

install(
  PROGRAMS
    ${PROJECT_SOURCE_DIR}/shobot_docking/node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME shobot_docking_node
)

install(
  FILES
    ${PROJECT_SOURCE_DIR}/shobot_docking/node.py
  DESTINATION ${Python3_SITEARCH}/shobot_docking
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/config)
  install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
