cmake_minimum_required(VERSION 3.5)
project(tr_real)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch
  configs
  maps
  DESTINATION share/${PROJECT_NAME})

ament_package()
