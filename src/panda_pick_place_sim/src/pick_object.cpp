#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>              // ← 수정
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/gripper_command.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
using GripperCommand     = control_msgs::action::GripperCommand;
using GoalHandleGripper  = rclcpp_action::ClientGoalHandle<GripperCommand>;

// gripper가 항상 아래를 향하도록 orientation 고정
geometry_msgs::msg::Pose make_pose(double x, double y, double z)
{
  geometry_msgs::msg::Pose p;
  p.position.x = x;
  p.position.y = y;
  p.position.z = z;
  tf2::Quaternion q;
  q.setRPY(-M_PI, 0.0, -M_PI/4.0);
  p.orientation = tf2::toMsg(q);
  return p;
}

// gripper open/close
void command_gripper_action(rclcpp::Node::SharedPtr node,
                            double position,
                            const std::string &label)
{
  auto client = rclcpp_action::create_client<GripperCommand>(
      node, "/panda_hand_controller/gripper_cmd");
  if (!client->wait_for_action_server(std::chrono::seconds(3))) {
    RCLCPP_ERROR(node->get_logger(),
                 "Gripper action server unavailable");
    return;
  }
  auto goal = GripperCommand::Goal();
  goal.command.position   = position;
  goal.command.max_effort  = 10.0;
  auto options = rclcpp_action::Client<GripperCommand>::SendGoalOptions();
  options.result_callback = [node,label](
      const GoalHandleGripper::WrappedResult &res) {
    if (res.code == rclcpp_action::ResultCode::SUCCEEDED)
      RCLCPP_INFO(node->get_logger(),
                  "Gripper %s succeeded", label.c_str());
    else
      RCLCPP_WARN(node->get_logger(),
                  "Gripper %s failed",    label.c_str());
  };
  client->async_send_goal(goal, options);
  rclcpp::sleep_for(std::chrono::seconds(2));
}

// 일반 pose 계획 + 실행
bool move_to_pose(MoveGroupInterface &arm,
                  const geometry_msgs::msg::Pose &target,
                  const std::string &name)
{
  arm.setPoseTarget(target);
  MoveGroupInterface::Plan plan;
  if (arm.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(arm.getNode()->get_logger(),
                 "Plan to %s failed", name.c_str());
    return false;
  }
  if (arm.execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(arm.getNode()->get_logger(),
                 "Execute to %s failed", name.c_str());
    return false;
  }
  RCLCPP_INFO(arm.getNode()->get_logger(),
              "Success: %s", name.c_str());
  return true;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("test_grasp");
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  std::thread([&exec](){ exec.spin(); }).detach();

  MoveGroupInterface arm(node, "panda_arm");

  // planning frame 확인 & 고정
  std::string pf = arm.getPlanningFrame();
  arm.setPoseReferenceFrame(pf);
  RCLCPP_INFO(node->get_logger(),
              "Planning frame: %s", pf.c_str());

  arm.setPlanningTime(3.0);
  arm.setNumPlanningAttempts(10);
  arm.allowReplanning(true);
  arm.setMaxVelocityScalingFactor(0.5);
  arm.setMaxAccelerationScalingFactor(0.5);
  arm.setGoalOrientationTolerance(0.05);

  // 1) pre-grasp
  RCLCPP_INFO(node->get_logger(), "Step 1: pre-grasp");
  if (!move_to_pose(arm, make_pose(0.5,0.0,0.6), "pre-grasp"))
    return 1;

  // 2) open gripper
  RCLCPP_INFO(node->get_logger(), "Step 2: open gripper");
  command_gripper_action(node, 0.035, "open");

  // 3) grasp pose
  RCLCPP_INFO(node->get_logger(), "Step 3: grasp");
  if (!move_to_pose(arm, make_pose(0.5,0.0,0.41), "grasp"))
    return 1;

  // 4) close gripper
  RCLCPP_INFO(node->get_logger(), "Step 4: close gripper");
  command_gripper_action(node, 0.02, "close");

  // 5) attach box
  RCLCPP_INFO(node->get_logger(), "Step 5: attach box");
  arm.attachObject("box", arm.getEndEffectorLink());
  rclcpp::sleep_for(std::chrono::milliseconds(500));

  // 6) lift straight up via Cartesian, 충돌 무시
  RCLCPP_INFO(node->get_logger(), "Step 6: lift straight up");
  auto start = arm.getCurrentPose().pose;
  std::vector<geometry_msgs::msg::Pose> wps = { start };
  auto up = start;
  up.position.z += 0.15;
  wps.push_back(up);

  moveit_msgs::msg::RobotTrajectory traj;
  double frac = arm.computeCartesianPath(
      wps, 0.01, /*jump*/0.0, traj,
      /*avoid_collisions*/false);

  if (frac < 0.9) {
    RCLCPP_ERROR(node->get_logger(),
                 "Cartesian lift only %.0f%%", frac*100.0);
    return 1;
  }
  MoveGroupInterface::Plan cart_plan;
  cart_plan.trajectory = traj;
  if (arm.execute(cart_plan) !=
      moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(),
                 "Execute Cartesian lift failed");
    return 1;
  }

  RCLCPP_INFO(node->get_logger(),
              "Pick-and-lift completed");
  rclcpp::shutdown();
  return 0;
}
