cmake_minimum_required(VERSION 3.8)
project(nav2_pose_select)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)

# 실행 파일 생성
add_executable(nav2_pose_select_node src/nav2_pose_select.cpp)
ament_target_dependencies(nav2_pose_select_node rclcpp geometry_msgs tf2)

# 설치 규칙: 이 부분이 있어야 ros2 run이 찾을 수 있습니다.
install(TARGETS nav2_pose_select_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()