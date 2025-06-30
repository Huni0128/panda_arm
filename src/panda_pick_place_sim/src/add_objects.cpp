#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

using std::placeholders::_1;

void add_collision_objects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{
  // Define a table
  moveit_msgs::msg::CollisionObject table;
  table.id = "table";
  table.header.frame_id = "panda_link0";

  shape_msgs::msg::SolidPrimitive table_primitive;
  table_primitive.type = table_primitive.BOX;
  table_primitive.dimensions = {0.5, 1.5, 0.4};  // X, Y, Z

  geometry_msgs::msg::Pose table_pose;
  table_pose.position.x = 0.5;
  table_pose.position.y = 0.0;
  table_pose.position.z = 0.2;

  table.primitives.push_back(table_primitive);
  table.primitive_poses.push_back(table_pose);
  table.operation = table.ADD;

  // Define a box to pick
  moveit_msgs::msg::CollisionObject box;
  box.id = "box";
  box.header.frame_id = "panda_link0";

  shape_msgs::msg::SolidPrimitive box_primitive;
  box_primitive.type = box_primitive.BOX;
  box_primitive.dimensions = {0.04, 0.04, 0.04};

  geometry_msgs::msg::Pose box_pose;
  box_pose.position.x = 0.5;
  box_pose.position.y = 0.0;
  box_pose.position.z = 0.42;

  box.primitives.push_back(box_primitive);
  box.primitive_poses.push_back(box_pose);
  box.operation = box.ADD;

  // Define a place platform
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

  // Apply all objects
  planning_scene_interface.applyCollisionObjects({table, box, platform});
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("add_collision_objects_node");

  // Setup Planning Interface
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  RCLCPP_INFO(node->get_logger(), "Adding collision objects...");
  add_collision_objects(planning_scene_interface);
  RCLCPP_INFO(node->get_logger(), "Done.");

  rclcpp::sleep_for(std::chrono::seconds(3));  // Wait for RViz to reflect changes
  rclcpp::shutdown();
  return 0;
}
