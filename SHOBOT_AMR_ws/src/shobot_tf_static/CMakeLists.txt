cmake_minimum_required(VERSION 3.5)
project(shobot_tf_static)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  PROGRAMS
    ${PROJECT_SOURCE_DIR}/shobot_tf_static/static_tf_publisher_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME static_tf_publisher_node
)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
