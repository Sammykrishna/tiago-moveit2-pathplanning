#include <memory>
#include <thread>
#include <chrono>

#include "rclcpp/rclcpp.hpp"

#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "moveit_msgs/msg/collision_object.hpp"
#include "shape_msgs/msg/solid_primitive.hpp"
#include "geometry_msgs/msg/pose.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("collision_object_test");

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  std::thread spinner([&exec]() { exec.spin(); });

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = "base_link";
  collision_object.id = "box1";

  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions = {0.4, 0.2, 0.1};

  geometry_msgs::msg::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 0.6;
  box_pose.position.y = 0.0;
  box_pose.position.z = 0.2;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = moveit_msgs::msg::CollisionObject::ADD;

  RCLCPP_INFO(node->get_logger(), "Spawning collision object '%s'...", collision_object.id.c_str());

  planning_scene_interface.applyCollisionObject(collision_object);

  std::this_thread::sleep_for(std::chrono::seconds(2));

  RCLCPP_INFO(node->get_logger(), "Done.");
  exec.cancel();
  if (spinner.joinable()) spinner.join();
  rclcpp::shutdown();
  return 0;
}