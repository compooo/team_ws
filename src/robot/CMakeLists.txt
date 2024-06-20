cmake_minimum_required(VERSION 3.5)
project(robot)

# Set C++ standard to 14 if necessary for other parts of your package
set(CMAKE_CXX_STANDARD 14)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)

# Install Python scripts
install(PROGRAMS
  scripts/teleop_keyboard.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files (if you have any)
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install urdf files (if you have any)
install(DIRECTORY
  urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)
# Install urdf files (if you have any)
install(DIRECTORY
  worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)

# Install config files (if you have any)
install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Export dependencies
ament_export_dependencies(rclpy geometry_msgs)

# Enable linting and testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()