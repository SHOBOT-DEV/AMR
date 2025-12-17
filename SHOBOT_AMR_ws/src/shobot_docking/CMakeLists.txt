cmake_minimum_required(VERSION 3.5)
project(shobot_docking)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Dock.action"
  DEPENDENCIES geometry_msgs std_msgs
)

# Guard against multiple evaluations when CMake re-enters this file during reconfigure
if(NOT TARGET ament_cmake_python_copy_${PROJECT_NAME}
    AND NOT TARGET ament_cmake_python_symlink_${PROJECT_NAME})
  ament_python_install_package(${PROJECT_NAME})
endif()

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/config)
  install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
