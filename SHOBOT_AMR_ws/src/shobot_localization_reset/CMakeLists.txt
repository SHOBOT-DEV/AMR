cmake_minimum_required(VERSION 3.5)
project(shobot_localization_reset)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  PROGRAMS
    ${PROJECT_SOURCE_DIR}/shobot_localization_reset/localization_reset_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME localization_reset_node
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
