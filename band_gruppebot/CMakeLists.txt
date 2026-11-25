cmake_minimum_required(VERSION 3.8)
project(band_gruppebot)

find_package(ament_cmake REQUIRED)

# Install package data
install(DIRECTORY urdf meshes launch rviz config worlds models
  DESTINATION share/${PROJECT_NAME}
)

# ROS 2-node (Python): simulert celle
install(PROGRAMS
  scripts/celle.py
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
