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

add_library(${PROJECT_NAME} INTERFACE
  include/${PROJECT_NAME}/eigen_stl_containers.h
  include/${PROJECT_NAME}/eigen_stl_map_container.h
  include/${PROJECT_NAME}/eigen_stl_vector_container.h
)

target_link_libraries(${PROJECT_NAME} INTERFACE Eigen3::Eigen)
ament_export_targets(${PROJECT_NAME}Targets)
ament_export_dependencies(Eigen3)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_package()
