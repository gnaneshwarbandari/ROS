cmake_minimum_required(VERSION 2.8.3)

# Mention your package name

project(m4w_robot_gazebo)

find_package(catkin REQUIRED COMPONENTS
gazebo_ros
)

catkin_package(
 CATKIN_DEPENDS gazebo_ros
)

install(DIRECTORY launch worlds
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)