cmake_minimum_required(VERSION 3.5)
project(shobot_navigation_server)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Navigate.srv"
  DEPENDENCIES geometry_msgs std_msgs
)

# Avoid duplicate target errors if processed more than once
if(NOT TARGET ament_cmake_python_symlink_${PROJECT_NAME})
  ament_python_install_package(${PROJECT_NAME})
endif()

install(
  PROGRAMS
    ${PROJECT_SOURCE_DIR}/shobot_navigation_server/node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME shobot_navigation_server_node
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY srv DESTINATION share/${PROJECT_NAME})
