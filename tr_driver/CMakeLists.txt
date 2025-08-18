cmake_minimum_required(VERSION 3.8)
project(tr_driver)


if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  #add_compile_options(-Wall -Wextra -Wpedantic)
  add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
  )
endif()

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBLELY_COAPP REQUIRED liblely-coapp)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcutils REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Boost )
find_package(nlohmann_json REQUIRED)
find_package(tr_driver_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(include)

set(DEPENDENCIES
	"rclcpp"
  "rcutils"
	"std_msgs"	
  "nlohmann_json"
  "tr_driver_msgs"
  "sensor_msgs"
  "trajectory_msgs"
)

add_executable(driving_motor_node src/main.cpp src/driving_motor.cpp) 
ament_target_dependencies(driving_motor_node ${DEPENDENCIES} )
target_link_libraries(driving_motor_node ${LIBLELY_COAPP_LIBRARIES})

# add_executable(operator_motor_node src/main.cpp src/operator_motor.cpp) 
# ament_target_dependencies(operator_motor_node ${DEPENDENCIES})
# target_link_libraries(operator_motor_node ${LIBLELY_COAPP_LIBRARIES})

install(
  TARGETS 
  driving_motor_node
  # operator_motor_node
  DESTINATION lib/${PROJECT_NAME})

install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

install(
    DIRECTORY config/
    DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
