cmake_minimum_required(VERSION 3.10.2)
project(panda_pick_place_sim)

# 필수 패키지 찾기
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)

# 실행 파일 정의
add_executable(add_objects src/add_objects.cpp)
ament_target_dependencies(add_objects
  rclcpp
  moveit_ros_planning_interface
  moveit_msgs
  geometry_msgs
  shape_msgs
)

# 설치 설정
install(TARGETS
  add_objects
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})

ament_package()
