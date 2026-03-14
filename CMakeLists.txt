cmake_minimum_required(VERSION 3.8)
project(pointcloud_to_occupancy_grid)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io segmentation)

add_definitions(${PCL_DEFINITIONS})

add_library(pointcloud_to_occupancy_grid_core
  src/subgrid.cpp
  src/occupancy_map.cpp
  src/dataset_io.cpp
  src/offline_map_builder.cpp
  src/map_export.cpp
)

target_include_directories(pointcloud_to_occupancy_grid_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

target_link_libraries(pointcloud_to_occupancy_grid_core
  ${PCL_LIBRARIES}
)

ament_target_dependencies(pointcloud_to_occupancy_grid_core
  rclcpp
)

add_executable(generate_pgm_from_keyframes
  src/generate_pgm_from_keyframes.cpp
)

target_link_libraries(generate_pgm_from_keyframes
  pointcloud_to_occupancy_grid_core
)

ament_target_dependencies(generate_pgm_from_keyframes
  rclcpp
)

install(TARGETS
  pointcloud_to_occupancy_grid_core
  generate_pgm_from_keyframes
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
