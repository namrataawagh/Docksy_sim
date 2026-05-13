cmake_minimum_required(VERSION 3.8)
project(my_robot_navigation)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# Install Python package (my_robot_navigation/ directory)
ament_python_install_package(${PROJECT_NAME})

# Install executables (RENAME drops .py so ros2 run/launch finds them by name)
install(PROGRAMS
  my_robot_navigation/parking_mission_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME parking_mission_node
)

install(PROGRAMS
  scripts/parking_manager.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME parking_manager
)

# Install config and launch directories
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
