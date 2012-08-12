cmake_minimum_required(VERSION 2.8)
project(eigen_stl_containers)

find_package(catkin REQUIRED)
catkin_stack()

catkin_project(${PROJECT_NAME} INCLUDE_DIRS include)

install(DIRECTORY include/
        DESTINATION include
        FILES_MATCHING PATTERN "*.h")
