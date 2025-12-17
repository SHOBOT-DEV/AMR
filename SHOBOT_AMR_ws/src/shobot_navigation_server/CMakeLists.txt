cmake_minimum_required(VERSION 3.5)
project(shobot_navigation_server)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(std_srvs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Navigate.srv"
  DEPENDENCIES geometry_msgs std_msgs
)

# Prevent duplicated custom targets when CMake re-enters this file
if(NOT TARGET ament_cmake_python_copy_${PROJECT_NAME}
    AND NOT TARGET ament_cmake_python_symlink_${PROJECT_NAME})
  ament_python_install_package(${PROJECT_NAME})
endif()

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
