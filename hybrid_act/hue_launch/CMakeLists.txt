cmake_minimum_required(VERSION 2.8.3)
project(hue_launch)

find_package(catkin REQUIRED)

catkin_package()

# Install targets
install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
