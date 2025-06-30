#pragma once
#include <rclcpp/rclcpp.hpp>
#include <control_msgs/action/gripper_command.hpp>

// position: 그리퍼 목표 폭, label: 로그 라벨
void commandGripperAction(
  rclcpp::Node::SharedPtr node,
  double position,
  const std::string &label,
  const std::string &action_topic = "/panda_hand_controller/gripper_cmd");
