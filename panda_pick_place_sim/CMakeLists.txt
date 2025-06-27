cmake_minimum_required(VERSION 3.10.2)
project(panda_pick_place_sim)

find_package(ament_cmake REQUIRED)

# 패키지 설치 설정
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})

ament_package()
