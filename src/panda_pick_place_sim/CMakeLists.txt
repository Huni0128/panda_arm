cmake_minimum_required(VERSION 3.10.2)
project(panda_pick_place_sim)

# ament / ROS2
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# MoveIt & 메시지
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# include 경로 지정
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# --- 모듈 라이브러리 정의 ---
add_library(pick_place_modules
  src/pose_utils.cpp
  src/gripper_control.cpp
  src/motion_planner.cpp
)
ament_target_dependencies(pick_place_modules
  rclcpp
  moveit_ros_planning_interface
  tf2_geometry_msgs
  control_msgs
  geometry_msgs
)

# --- add_objects 라이브러리 정의 (add_colored_objects 함수 제공) ---
add_library(scene_setup
  src/add_objects.cpp
)
ament_target_dependencies(scene_setup
  rclcpp
  moveit_ros_planning_interface
  moveit_msgs
  geometry_msgs
  shape_msgs
)
target_include_directories(scene_setup PUBLIC include)

# --- main 노드 (모듈 + scene 사용) ---
add_executable(main src/main.cpp)
ament_target_dependencies(main
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  moveit_msgs
  control_msgs
)
target_link_libraries(main
  pick_place_modules
  scene_setup
)

# --- 설치 설정 ---
install(TARGETS
  main
  pick_place_modules
  scene_setup
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rviz   DESTINATION share/${PROJECT_NAME})

ament_package()
