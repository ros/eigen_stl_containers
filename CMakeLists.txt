cmake_minimum_required(VERSION 2.8.3)
project(eigen_stl_containers)

find_package(catkin REQUIRED)
catkin_package(
  INCLUDE_DIRS include
  DEPENDS Eigen)

install(DIRECTORY include/
        DESTINATION include
        FILES_MATCHING PATTERN "*.h")
