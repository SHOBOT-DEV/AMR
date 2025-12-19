cmake_minimum_required(VERSION 3.5)
project(shobot_sensors)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

install(
  PROGRAMS
    ${PROJECT_SOURCE_DIR}/shobot_sensors/imu_republisher_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME imu_republisher_node
)

install(
  PROGRAMS
    ${PROJECT_SOURCE_DIR}/shobot_sensors/wheel_odometry_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME wheel_odometry_node
)

install(
  PROGRAMS
    ${PROJECT_SOURCE_DIR}/shobot_sensors/battery_monitor.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME battery_monitor_node
)

ament_package()
