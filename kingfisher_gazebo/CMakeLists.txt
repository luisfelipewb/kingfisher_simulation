cmake_minimum_required(VERSION 3.8)
project(kingfisher_gazebo)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(gz-math7 REQUIRED)
find_package(sdformat14 REQUIRED)

# lib/ on GZ_SIM_SYSTEM_PLUGIN_PATH, for libApparentWind.so
ament_environment_hooks("hooks/resource_paths.dsv.in")
ament_environment_hooks("hooks/resource_paths.sh")

add_library(ApparentWind SHARED src/ApparentWind.cc)
target_link_libraries(ApparentWind PUBLIC
  gz-sim8::core
  gz-plugin2::register
  gz-transport13::core
  gz-msgs10::core
  gz-math7::core
  sdformat14::sdformat14)
install(TARGETS ApparentWind DESTINATION lib)

install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_wind_model test/test_wind_model.cc)
  target_include_directories(test_wind_model PRIVATE src)
  target_link_libraries(test_wind_model gz-math7::core)
endif()

ament_package()
