cmake_minimum_required(VERSION 3.8)
project(tr_driver_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotorStatus.msg"
  "msg/MotorDriver.msg"
  "msg/ChannelStatus.msg"
  "msg/SafetyStatus.msg"
  "msg/WheelControl.msg"
  "msg/MotorState.msg"
  "msg/PgvData.msg"
  "msg/PgvDirCmd.msg"
  "srv/MotorRunPosition.srv"
  "srv/MotorRunVelocity.srv"
  "srv/MotorStopHoming.srv"
  "srv/MotorSendVar.srv"
  DEPENDENCIES std_msgs
)

install(
  DIRECTORY "msg"
  DESTINATION "share/${PROJECT_NAME}/msg"
)

ament_package()
