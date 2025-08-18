cmake_minimum_required(VERSION 3.5)
project(tr_kinematics_differential2)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# ###########
# ## Build ##
# ###########

ament_export_dependencies(std_msgs tf nav_msgs sensor_msgs)

ament_export_include_directories(common/include)

add_library(${PROJECT_NAME} common/src/DiffDrive2WKinematics.cpp)


ament_export_libraries(${PROJECT_NAME})

add_executable(tr_differential_node ros/src/tr_kinematics_differential_node.cpp)

ament_target_dependencies(tr_differential_node
  rclcpp
  tf2_ros
  sensor_msgs
  geometry_msgs
  std_msgs
  nav_msgs
  trajectory_msgs
  tf2_geometry_msgs)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  tf2_ros
  sensor_msgs
  geometry_msgs
  std_msgs
  nav_msgs
  trajectory_msgs
  tf2_geometry_msgs)

target_link_libraries(tr_differential_node ${PROJECT_NAME})

install(DIRECTORY common/include/
   DESTINATION common/include/
)

install(TARGETS tr_differential_node
DESTINATION lib/${PROJECT_NAME}
	)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()