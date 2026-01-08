cmake_minimum_required(VERSION 3.5)
project(shobot_costmap_safety_layer)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  PROGRAMS
    ${PROJECT_SOURCE_DIR}/shobot_costmap_safety_layer/node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME shobot_costmap_safety_layer_node
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/config)
  install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
endif()

ament_package()
