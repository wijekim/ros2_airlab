cmake_minimum_required(VERSION 3.8)
project(my_nav2_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 의존성 패키지 찾기
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# 초기/목표 위치 발행 노드 (00point.cpp) 빌드 설정
add_executable(pose_publisher src/00point.cpp)
ament_target_dependencies(pose_publisher
  rclcpp
  geometry_msgs
  tf2
  tf2_geometry_msgs
)

# 웨이포인트 팔로워 노드 (waypointfollow.cpp) 빌드 설정
add_executable(waypoint_follower src/waypointfollow.cpp)
ament_target_dependencies(waypoint_follower
  rclcpp
  rclcpp_action
  geometry_msgs
  nav2_msgs
  tf2
  tf2_geometry_msgs
)

# 설치 경로 설정 (ros2 run으로 실행할 수 있게 해줌)
install(TARGETS
  pose_publisher
  waypoint_follower
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()