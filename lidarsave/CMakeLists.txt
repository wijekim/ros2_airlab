cmake_minimum_required(VERSION 3.16)
project(lidarsave)


if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

add_executable(lidarsave src/lidarsave.cpp)
ament_target_dependencies(
  lidarsave 
  rclcpp 
  sensor_msgs 
  OpenCV
  geometry_msgs
  cv_bridge
  
)

add_executable(lidarsave_simul src/lidarsave_simul.cpp)
ament_target_dependencies(
  lidarsave_simul 
  rclcpp 
  sensor_msgs 
  OpenCV
  geometry_msgs
  cv_bridge
  
)

install(TARGETS 
  lidarsave
  lidarsave_simul
  DESTINATION lib/${PROJECT_NAME})

ament_package()