// pick_object.cpp: MoveIt2를 이용한 Panda 로봇팔 픽 동작 노드

#include <rclcpp/rclcpp.hpp>                                 // ROS2 기본 헤더
#include <moveit/move_group_interface/move_group_interface.hpp>  // MoveGroup 인터페이스
#include <thread>                                            // std::thread
#include <chrono>                                            // std::chrono

#include "panda_pick_place_sim/pose_utils.hpp"               // makePose() 유틸
#include "panda_pick_place_sim/gripper_control.hpp"          // commandGripperAction() 유틸
#include "panda_pick_place_sim/motion_planner.hpp"           // moveToPose(), cartesianLift() 유틸

int main(int argc, char **argv)
{
  // 1) ROS2 초기화 및 노드 생성
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("pick_object");

  // 2) SingleThreadedExecutor에 노드 등록 후 스핀 스레드 실행
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  std::thread([&exec](){ exec.spin(); }).detach();

  // 3) MoveIt! MoveGroupInterface 초기화 ("panda_arm" 그룹)
  moveit::planning_interface::MoveGroupInterface arm(node, "panda_arm");

  // 4) 기본 설정
  const std::string planning_frame = arm.getPlanningFrame();
  arm.setPoseReferenceFrame(planning_frame);
  RCLCPP_INFO(node->get_logger(), "Planning frame: %s", planning_frame.c_str());

  arm.setPlanningTime(3.0);                   // 계획 허용 시간(s)
  arm.setNumPlanningAttempts(10);             // 계획 시도 횟수
  arm.allowReplanning(true);                  // 실패 시 재계획 허용
  arm.setMaxVelocityScalingFactor(0.5);       // 최대 속도 비율
  arm.setMaxAccelerationScalingFactor(0.5);   // 최대 가속도 비율
  arm.setGoalOrientationTolerance(0.05);      // 목표 자세 허용 오차(rad)

  // ==== 픽 동작 순서 ====

  // 5) Pre-grasp 포즈로 이동
  RCLCPP_INFO(node->get_logger(), "Step 1: pre-grasp");
  if (!moveToPose(arm, makePose(0.5, 0.0, 0.6), "pre-grasp")) {
    RCLCPP_ERROR(node->get_logger(), "Pre-grasp failed");
    return 1;
  }

  // 6) 그리퍼 열기
  RCLCPP_INFO(node->get_logger(), "Step 2: open gripper");
  commandGripperAction(node, 0.035, "open");

  // 7) Grasp 포즈로 이동
  RCLCPP_INFO(node->get_logger(), "Step 3: grasp");
  if (!moveToPose(arm, makePose(0.5, 0.0, 0.41), "grasp")) {
    RCLCPP_ERROR(node->get_logger(), "Grasp move failed");
    return 1;
  }

  // 8) 그리퍼 닫기
  RCLCPP_INFO(node->get_logger(), "Step 4: close gripper");
  commandGripperAction(node, 0.02, "close");

  // 9) 객체 Attach (Collision 객체로 등록)
  RCLCPP_INFO(node->get_logger(), "Step 5: attach object");
  arm.attachObject("box", arm.getEndEffectorLink());
  rclcpp::sleep_for(std::chrono::milliseconds(500));  // 잠시 대기

  // 10) 카르테시안 리프트 (수직으로 0.15m 상승)
  RCLCPP_INFO(node->get_logger(), "Step 6: lift straight up");
  if (!cartesianLift(arm, 0.15)) {
    RCLCPP_ERROR(node->get_logger(), "Cartesian lift failed");
    return 1;
  }

  // 11) 완료 로그 및 종료
  RCLCPP_INFO(node->get_logger(), "Pick-and-lift completed");
  rclcpp::shutdown();
  return 0;
}
