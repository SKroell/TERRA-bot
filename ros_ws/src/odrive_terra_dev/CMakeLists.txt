cmake_minimum_required(VERSION 3.5)
project(odrive_terra_dev)

# Find dependencies
#find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
#find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

# Glob source files
file(GLOB_RECURSE PROJECT_SOURCES launch/*.launch.py config/*.yaml)

# Install
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.launch.py" PATTERN "*.yaml"
)

ament_python_install_package(${PROJECT_NAME})


# Python scripts
install(PROGRAMS
  trackers/follow_detection.py
  trackers/detect_human.py
  trackers/detect_human_with_wave.py
  trackers/detect_human_yolo.py
  trackers/detect_aruco.py
  DESTINATION lib/${PROJECT_NAME}
)

# Package setup
ament_package()