cmake_minimum_required(VERSION 3.10.2)
project(panda_pick_place_sim)

# 필수 패키지 찾기
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(control_msgs REQUIRED)  # ← 추가됨

# add_objects 노드
add_executable(add_objects src/add_objects.cpp)
ament_target_dependencies(add_objects
  rclcpp
  moveit_ros_planning_interface
  moveit_msgs
  geometry_msgs
  shape_msgs
)

# pick_object 노드 추가
add_executable(pick_object src/pick_object.cpp)
ament_target_dependencies(pick_object
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  moveit_msgs
  control_msgs  # ← 추가됨
)

# pick_debug 노드
add_executable(pick_debug src/pick_debug.cpp)
ament_target_dependencies(pick_debug 
  rclcpp
  moveit_ros_planning_interface
)

# 설치 설정
install(TARGETS
  add_objects
  pick_object
  pick_debug
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})

ament_package()
