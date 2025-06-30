#pragma once
#include <geometry_msgs/msg/pose.hpp>

// gripper가 항상 아래를 향하도록 orientation 고정
geometry_msgs::msg::Pose makePose(double x, double y, double z);
