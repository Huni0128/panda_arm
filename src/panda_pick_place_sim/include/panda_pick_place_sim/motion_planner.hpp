#pragma once

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/utils/moveit_error_code.hpp>            // MoveItErrorCode 래퍼
#include <geometry_msgs/msg/pose.hpp>
#include <string>

// pose 목표로 계획 + 실행
bool moveToPose(
  moveit::planning_interface::MoveGroupInterface &arm,
  const geometry_msgs::msg::Pose &target,
  const std::string &name);

// Cartesian으로 위로 들어올리기
bool cartesianLift(
  moveit::planning_interface::MoveGroupInterface &arm,
  double lift_distance);
