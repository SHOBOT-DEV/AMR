cmake_minimum_required(VERSION 3.8)
project(shobot_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch urdf rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
