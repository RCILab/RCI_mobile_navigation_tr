cmake_minimum_required(VERSION 3.8)
project(tr_sim)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch
  configs
  maps
  worlds
  components
  models
  DESTINATION share/${PROJECT_NAME})

ament_package()
