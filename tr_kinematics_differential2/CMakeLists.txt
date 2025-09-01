cmake_minimum_required(VERSION 3.5)
project(tr_kinematics_differential2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/DiffDrive2WKinematics.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  "rclcpp"
  "geometry_msgs"
  "nav_msgs"
  "sensor_msgs"
  "std_msgs"
  "trajectory_msgs"
  "tf2_ros"
  "tf2_geometry_msgs"
)

add_executable(tr_differential_node src/tr_kinematics_differential_node.cpp)

target_include_directories(tr_differential_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(tr_differential_node
  "rclcpp"
  "geometry_msgs"
  "nav_msgs"
  "sensor_msgs"
  "std_msgs"
  "trajectory_msgs"
  "tf2_ros"
)

target_link_libraries(tr_differential_node ${PROJECT_NAME})

install(TARGETS
  ${PROJECT_NAME}
  tr_differential_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(
  "rclcpp"
  "geometry_msgs"
  "nav_msgs"
  "sensor_msgs"
  "std_msgs"
  "trajectory_msgs"
  "tf2_ros"
)

ament_package()