cmake_minimum_required(VERSION 3.5)
project(shobot_status_aggregator)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/config)
  install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
endif()

# Install Python entrypoint so `ros2 run`/launch can find it
install(
  PROGRAMS
    ${CMAKE_CURRENT_SOURCE_DIR}/shobot_status_aggregator/node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME shobot_status_aggregator_node
)

ament_package()
