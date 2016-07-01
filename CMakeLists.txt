cmake_minimum_required(VERSION 2.8.3)
project(eigen_stl_containers)

find_package(catkin REQUIRED)

# we don't build anything here, but in order to export the right
# build flags in catkin_package(), we still have to find Eigen3 here
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  DEPENDS EIGEN3)

install(DIRECTORY include/
        DESTINATION include
        FILES_MATCHING PATTERN "*.h")
