#include <rclcpp/rclcpp.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/object_color.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose.hpp>

void add_colored_objects(rclcpp::Node::SharedPtr node)
{
  // 퍼블리셔 설정
  auto planning_scene_publisher =
      node->create_publisher<moveit_msgs::msg::PlanningScene>("/planning_scene", 10);

  // ----- 테이블 생성 -----
  moveit_msgs::msg::CollisionObject table;
  table.id = "table";
  table.header.frame_id = "panda_link0";

  shape_msgs::msg::SolidPrimitive table_primitive;
  table_primitive.type = table_primitive.BOX;
  table_primitive.dimensions = {0.5, 1.3, 0.3};

  geometry_msgs::msg::Pose table_pose;
  table_pose.position.x = 0.5;
  table_pose.position.y = 0.0;
  table_pose.position.z = 0.1;

  table.primitives.push_back(table_primitive);
  table.primitive_poses.push_back(table_pose);
  table.operation = table.ADD;

  // ----- 박스 생성 -----
  moveit_msgs::msg::CollisionObject box;
  box.id = "box";
  box.header.frame_id = "panda_link0";

  shape_msgs::msg::SolidPrimitive box_primitive;
  box_primitive.type = box_primitive.BOX;
  box_primitive.dimensions = {0.06, 0.06, 0.06};

  geometry_msgs::msg::Pose box_pose;
  box_pose.position.x = 0.5;
  box_pose.position.y = 0.0;
  box_pose.position.z = 0.30;

  box.primitives.push_back(box_primitive);
  box.primitive_poses.push_back(box_pose);
  box.operation = box.ADD;

  // ----- 플랫폼 생성 -----
  moveit_msgs::msg::CollisionObject platform;
  platform.id = "platform";
  platform.header.frame_id = "panda_link0";

  shape_msgs::msg::SolidPrimitive platform_primitive;
  platform_primitive.type = platform_primitive.BOX;
  platform_primitive.dimensions = {0.1, 0.1, 0.02};

  geometry_msgs::msg::Pose platform_pose;
  platform_pose.position.x = 0.0;
  platform_pose.position.y = -0.4;
  platform_pose.position.z = 0.42;

  platform.primitives.push_back(platform_primitive);
  platform.primitive_poses.push_back(platform_pose);
  platform.operation = platform.ADD;

  // ----- 색상 설정 -----
  moveit_msgs::msg::ObjectColor table_color;
  table_color.id = "table";
  table_color.color.r = 0.0;
  table_color.color.g = 0.0;
  table_color.color.b = 0.0;
  table_color.color.a = 1.0;

  moveit_msgs::msg::ObjectColor box_color;
  box_color.id = "box";
  box_color.color.r = 1.0;
  box_color.color.g = 1.0;
  box_color.color.b = 0.0;
  box_color.color.a = 1.0;

  moveit_msgs::msg::ObjectColor platform_color;
  platform_color.id = "platform";
  platform_color.color.r = 0.0;
  platform_color.color.g = 0.0;
  platform_color.color.b = 0.0;
  platform_color.color.a = 1.0;

  // ----- PlanningScene 메시지 구성 -----
  moveit_msgs::msg::PlanningScene planning_scene;
  planning_scene.is_diff = true;
  planning_scene.world.collision_objects = {table, box, platform};
  planning_scene.object_colors = {table_color, box_color, platform_color};

  // 퍼블리시
  rclcpp::Rate rate(10);
  for (int i = 0; i < 10; ++i) {
    planning_scene_publisher->publish(planning_scene);
    rate.sleep();
  }
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("add_colored_objects_node");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();

  RCLCPP_INFO(node->get_logger(), "Publishing colored collision objects...");
  add_colored_objects(node);
  RCLCPP_INFO(node->get_logger(), "Done.");

  rclcpp::sleep_for(std::chrono::seconds(2));
  rclcpp::shutdown();
  return 0;
}
