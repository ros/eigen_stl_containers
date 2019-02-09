cmake_minimum_required(VERSION 3.5)
project(eigen_stl_containers)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
if(NOT EIGEN3_FOUND)
  # saucy does not have Eigen3, but have Eigen instead
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
endif()

# Note that eigen 3.2 (on Ubuntu Wily) only provides EIGEN3_INCLUDE_DIR,
# not EIGEN3_INCLUDE_DIRS, so we have to set the latter from the former.
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION include/${PROJECT_NAME}
        FILES_MATCHING PATTERN "*.h")

ament_export_include_directories(include ${EIGEN3_INCLUDE_DIRS})
ament_package()
