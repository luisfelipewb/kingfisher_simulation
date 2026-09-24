cmake_minimum_required(VERSION 3.8)
project(kingfisher_gazebo)
find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf)

ament_package()
