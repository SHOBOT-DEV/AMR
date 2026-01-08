cmake_minimum_required(VERSION 3.5)
project(shobot_api_bridge)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/config)
  install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
endif()
install(PROGRAMS scripts/api_bridge DESTINATION lib/${PROJECT_NAME})
install(
  PROGRAMS
    ${PROJECT_SOURCE_DIR}/shobot_api_bridge/mission_api_bridge.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME mission_api_bridge
)

ament_package()
