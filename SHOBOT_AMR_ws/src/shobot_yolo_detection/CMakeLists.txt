cmake_minimum_required(VERSION 3.5)
project(shobot_yolo_detection)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/config)
  install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
endif()

install(PROGRAMS
  scripts/shobot_yolo_detection_node
  scripts/shobot_realsense_camera_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
