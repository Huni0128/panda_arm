#include "panda_pick_place_sim/gripper_control.hpp"
#include <rclcpp_action/rclcpp_action.hpp>
#include <chrono>

// GripperCommand 액션 메시지와 GoalHandle 정의
using GripperCommand    = control_msgs::action::GripperCommand;
using GoalHandleGripper = rclcpp_action::ClientGoalHandle<GripperCommand>;

/**
 * @brief 그리퍼를 여닫는 액션을 호출하는 함수
 * 
 * @param node        RCLCPP 노드 핸들
 * @param position    그리퍼 목표 폭 (m 단위)
 * @param label       로그 출력 시 사용되는 라벨 (예: "open", "close")
 * @param action_topic 액션 서버 토픽 이름 (예: "/panda_hand_controller/gripper_cmd")
 */
void commandGripperAction(
    rclcpp::Node::SharedPtr node,
    double position,
    const std::string &label,
    const std::string &action_topic)
{
  // 1) 액션 클라이언트 생성
  auto client = rclcpp_action::create_client<GripperCommand>(
      node, action_topic);

  // 2) 액션 서버가 준비될 때까지 최대 3초 대기
  if (!client->wait_for_action_server(std::chrono::seconds(3))) {
    RCLCPP_ERROR(node->get_logger(),
                 "Gripper action server unavailable");
    return;
  }

  // 3) 그리퍼 액션 Goal 설정
  auto goal = GripperCommand::Goal();
  goal.command.position   = position;  // 목표 그리퍼 폭
  goal.command.max_effort = 10.0;      // 최대 토크/힘 설정

  // 4) 결과 콜백 옵션 정의
  auto options = rclcpp_action::Client<GripperCommand>::SendGoalOptions();
  options.result_callback =
    [node, label](const GoalHandleGripper::WrappedResult &res) {
      // 액션 결과 코드에 따라 로그 분기
      if (res.code == rclcpp_action::ResultCode::SUCCEEDED)
        RCLCPP_INFO(node->get_logger(),
                    "Gripper %s succeeded", label.c_str());
      else
        RCLCPP_WARN(node->get_logger(),
                    "Gripper %s failed", label.c_str());
    };

  // 5) 비동기(goal) 전송
  client->async_send_goal(goal, options);

  // 6) 액션 수행 대기 (간단히 sleep, 콜백으로 결과 처리)
  rclcpp::sleep_for(std::chrono::seconds(2));
}
