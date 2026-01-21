cmake_minimum_required(VERSION 3.8)
project(my_launch)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
# 런치 파일만 있다면 rclcpp, std_msgs는 사실 없어도 되지만, 
# 나중에 코드를 추가하실 거라면 남겨두셔도 됩니다.

# 설치 설정: launch, maps, urdf 폴더를 패키지의 share 디렉토리로 복사
install(DIRECTORY
  launch
  maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()