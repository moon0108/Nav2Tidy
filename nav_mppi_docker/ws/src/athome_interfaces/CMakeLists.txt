cmake_minimum_required(VERSION 3.8)
project(athome_interfaces)

# 임시 정의 -- 팀 원문 수령 시 이 패키지를 통째로 교체한다.

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/NavigateToGoal.action"
  DEPENDENCIES geometry_msgs nav_msgs)

ament_package()
