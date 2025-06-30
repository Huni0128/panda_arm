#include "panda_pick_place_sim/pose_utils.hpp"           // 포즈 생성 유틸 헤더
#include <tf2/LinearMath/Quaternion.h>                  // TF2 쿼터니언 클래스
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>      // tf2 ↔ geometry_msgs 변환 함수
#include <cmath>                                        // M_PI 상수

/**
 * @brief 지정한 위치(x, y, z)에 그리퍼가 항상 아래를 향하도록 고정된 orientation을 가진 Pose 생성
 *
 * @param x  x축 위치 (m)
 * @param y  y축 위치 (m)
 * @param z  z축 위치 (m)
 * @return  생성된 geometry_msgs::msg::Pose 객체
 */
geometry_msgs::msg::Pose makePose(double x, double y, double z) {
  // 1) 위치(position) 설정
  geometry_msgs::msg::Pose p;
  p.position.x = x;
  p.position.y = y;
  p.position.z = z;

  // 2) 쿼터니언 생성: roll = -π, pitch = 0, yaw = -π/4
  //    → 그리퍼가 항상 아래를 향하도록 orientation을 고정
  tf2::Quaternion q;
  q.setRPY(-M_PI, 0.0, -M_PI / 4.0);

  // 3) tf2 쿼터니언을 ROS geometry_msgs 형식으로 변환하여 저장
  p.orientation = tf2::toMsg(q);

  // 4) 완성된 Pose 반환
  return p;
}
