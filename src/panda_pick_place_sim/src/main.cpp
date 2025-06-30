// pick_object.cpp: MoveIt2를 이용한 Panda 로봇팔 픽앤플레이스(Pick-and-Place) 노드

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <thread>
#include <chrono>
#include <vector>

#include "panda_pick_place_sim/pose_utils.hpp"         // makePose(x, y, z)
#include "panda_pick_place_sim/gripper_control.hpp"     // commandGripperAction(...)
#include "panda_pick_place_sim/motion_planner.hpp"      // moveToPose(), cartesianLift()
#include "panda_pick_place_sim/add_objects.hpp"         // add_colored_objects() - scene 초기화

int main(int argc, char **argv)
{
  // 1) ROS2 초기화 및 노드 생성
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("pick_object");

  // 2) ROS Executor 실행 (non-blocking)
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  std::thread([&exec]() { exec.spin(); }).detach();

  // 3) Scene 초기화 (테이블, 박스, 플랫폼 오브젝트 세팅 + 기존 box 삭제)
  RCLCPP_INFO(node->get_logger(), "Step 0: resetting scene...");
  add_colored_objects(node);
  rclcpp::sleep_for(std::chrono::seconds(1));  // scene 적용 대기

  // 4) MoveIt 인터페이스 초기화 (panda_arm 그룹 사용)
  moveit::planning_interface::MoveGroupInterface arm(node, "panda_arm");

  // 5) 로봇 기본 제약조건 및 옵션 설정
  const std::string planning_frame = arm.getPlanningFrame();
  arm.setPoseReferenceFrame(planning_frame);
  RCLCPP_INFO(node->get_logger(), "Planning frame: %s", planning_frame.c_str());

  arm.setPlanningTime(3.0);
  arm.setNumPlanningAttempts(10);
  arm.allowReplanning(true);
  arm.setMaxVelocityScalingFactor(0.5);
  arm.setMaxAccelerationScalingFactor(0.5);
  arm.setGoalOrientationTolerance(0.05);

  // ========================
  // ===== Pick 과정 시작 =====
  // ========================

  // Step 1: 집기 전 위치로 이동 (pre-grasp pose)
  RCLCPP_INFO(node->get_logger(), "Step 1: pre-grasp");
  if (!moveToPose(arm, makePose(0.5, 0.0, 0.6), "pre-grasp")) return 1;

  // Step 2: 그리퍼 열기
  RCLCPP_INFO(node->get_logger(), "Step 2: open gripper");
  commandGripperAction(node, 0.035, "open");

  // Step 3: 집을 위치로 이동 (grasp pose)
  RCLCPP_INFO(node->get_logger(), "Step 3: grasp");
  if (!moveToPose(arm, makePose(0.5, 0.0, 0.41), "grasp")) return 1;

  // Step 4: 그리퍼 닫기 (물체 잡기)
  RCLCPP_INFO(node->get_logger(), "Step 4: close gripper");
  commandGripperAction(node, 0.02, "close");

  // Step 5: 물체 붙이기 (충돌 무시 링크 지정 포함)
  RCLCPP_INFO(node->get_logger(), "Step 5: attach object");
  std::vector<std::string> touch_links = arm.getLinkNames();
  arm.attachObject("box", arm.getEndEffectorLink(), touch_links);
  rclcpp::sleep_for(std::chrono::milliseconds(500));

  // Step 6: 물체를 수직으로 들어올리기
  RCLCPP_INFO(node->get_logger(), "Step 6: lift straight up");
  if (!cartesianLift(arm, 0.15)) return 1;

  // ========================
  // ===== Place 과정 시작 =====
  // ========================

  // Step 7: 플랫폼 방향으로 이동 (카르테시안 경로 사용)
  RCLCPP_INFO(node->get_logger(), "Step 7: move to place");

  geometry_msgs::msg::Pose current_pose = arm.getCurrentPose().pose;
  geometry_msgs::msg::Pose place_pose = makePose(0.0, -0.4, 0.47);  // platform 위치

  std::vector<geometry_msgs::msg::Pose> waypoints{current_pose, place_pose};

  moveit_msgs::msg::RobotTrajectory trajectory;
  double fraction = arm.computeCartesianPath(waypoints, 0.01, trajectory, false);  // avoid_collisions=false

  if (fraction < 0.9) {
    RCLCPP_WARN(node->get_logger(), "Cartesian path incomplete: %.2f%%", fraction * 100.0);
  } else {
    moveit::planning_interface::MoveGroupInterface::Plan place_plan;
    place_plan.trajectory = trajectory;
    arm.execute(place_plan);
  }

  // Step 9: 그리퍼 열기 (물체 놓기)
  RCLCPP_INFO(node->get_logger(), "Step 9: release object");
  commandGripperAction(node, 0.035, "open");

  // Step 10: 물체 detach (이제 로봇과 연결 끊김)
  RCLCPP_INFO(node->get_logger(), "Step 10: detach object");
  arm.detachObject("box");

  // Step 11: 위로 잠시 후퇴
  RCLCPP_INFO(node->get_logger(), "Step 11: retreat");
  moveToPose(arm, makePose(0.0, -0.4, 0.6), "retreat");

  // Step 12: 초기 위치(pre-grasp pose)로 복귀
  RCLCPP_INFO(node->get_logger(), "Step 12: return to initial position");
  if (!moveToPose(arm, makePose(0.5, 0.0, 0.6), "home")) {
    RCLCPP_WARN(node->get_logger(), "Return to initial pose failed");
  }

  // 종료
  RCLCPP_INFO(node->get_logger(), "Pick-and-place demo complete");
  rclcpp::shutdown();
  return 0;
}
