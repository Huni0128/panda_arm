// motion_planner.cpp: MoveIt!을 이용한 로봇 팔의 이동 및 카르테시안 궤적 제어 기능 구현

#include "panda_pick_place_sim/motion_planner.hpp"                          // 모듈 헤더
#include <moveit/move_group_interface/move_group_interface.hpp>           // MoveGroupInterface
#include <moveit_msgs/msg/move_it_error_codes.hpp>                       // 계획/실행 결과 코드
#include <moveit_msgs/msg/robot_trajectory.hpp>                          // RobotTrajectory 메시지
#include <geometry_msgs/msg/pose.hpp>                                    // Pose 메시지
#include <rclcpp/rclcpp.hpp>                                             // ROS2 로깅, 시간
#include <vector>                                                        // std::vector

/**
 * @brief 지정한 목표 포즈로 로봇 팔을 이동(계획+실행)한다.
 *
 * @param arm     MoveGroupInterface 객체 (로봇 팔 인터페이스)
 * @param target  목표 위치/자세 (geometry_msgs::msg::Pose)
 * @param name    로그 및 오류 메시지 식별용 이름
 * @return 성공하면 true, 실패하면 false
 */
bool moveToPose(
    moveit::planning_interface::MoveGroupInterface &arm,
    const geometry_msgs::msg::Pose &target,
    const std::string &name)
{
  // 1) 목표 포즈 설정
  arm.setPoseTarget(target);

  // 2) 경로(plan) 객체 생성
  moveit::planning_interface::MoveGroupInterface::Plan plan;

  // 3) 경로 계획 요청
  auto result = arm.plan(plan);
  if (result != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
    // 계획 실패 시 오류 로그 출력 후 종료
    RCLCPP_ERROR(arm.getNode()->get_logger(),
                 "Plan to %s failed", name.c_str());
    return false;
  }

  // 4) 계획된 경로 실행
  auto exec_res = arm.execute(plan);
  if (exec_res != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
    // 실행 실패 시 오류 로그 출력 후 종료
    RCLCPP_ERROR(arm.getNode()->get_logger(),
                 "Execute to %s failed", name.c_str());
    return false;
  }

  // 5) 성공 로그
  RCLCPP_INFO(arm.getNode()->get_logger(),
              "Success: %s", name.c_str());
  return true;
}

/**
 * @brief 현재 위치에서 수직으로 일정 거리만큼 카르테시안 경로로 들어올린다.
 *
 * @param arm            MoveGroupInterface 객체
 * @param lift_distance  들어올릴 거리 (미터)
 * @return 성공하면 true, 실패하면 false
 */
bool cartesianLift(
    moveit::planning_interface::MoveGroupInterface &arm,
    double lift_distance)
{
  // 1) 현재 엔드 이펙터 포즈 획득
  auto start = arm.getCurrentPose().pose;

  // 2) 시작점(wps[0]) 설정
  std::vector<geometry_msgs::msg::Pose> wps{start};

  // 3) 목표점 생성: z축으로 lift_distance만큼 상승
  auto up = start;
  up.position.z += lift_distance;
  wps.push_back(up);

  // 4) 카르테시안 궤적 생성
  moveit_msgs::msg::RobotTrajectory traj;
  double frac = arm.computeCartesianPath(
      wps,
      0.01,                           // eef_step: 엔드 이펙터 이동 간격(m)
      traj,
      /*avoid_collisions=*/false);    // 충돌 허용 여부

  // 5) 궤적 유효 비율 확인
  if (frac < 0.9) {
    RCLCPP_ERROR(arm.getNode()->get_logger(),
                 "Cartesian lift only %.0f%%", frac * 100.0);
    return false;
  }

  // 6) 생성된 궤적 실행
  moveit::planning_interface::MoveGroupInterface::Plan cart_plan;
  cart_plan.trajectory = traj;
  auto exec_res = arm.execute(cart_plan);
  if (exec_res != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
    RCLCPP_ERROR(arm.getNode()->get_logger(),
                 "Execute Cartesian lift failed");
    return false;
  }

  return true;
}
