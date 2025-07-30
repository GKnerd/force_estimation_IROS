cmake_minimum_required(VERSION 3.22)
project(panda_moveit2_config)

find_package(ament_cmake REQUIRED)
find_package(panda_urdf REQUIRED)

install(DIRECTORY config launch rviz ros2_controllers srdf DESTINATION share/${PROJECT_NAME})
ament_package()
