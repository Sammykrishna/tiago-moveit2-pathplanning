#include <memory>
#include <thread>
#include <chrono>
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"

#include "moveit_msgs/msg/collision_object.hpp"
#include "moveit_msgs/msg/attached_collision_object.hpp"
#include "shape_msgs/msg/solid_primitive.hpp"

using namespace std::chrono_literals;

static geometry_msgs::msg::Pose make_pose(double x, double y, double z)
{
  geometry_msgs::msg::Pose p;
  p.position.x = x;
  p.position.y = y;
  p.position.z = z;
  p.orientation.x = 0.0;
  p.orientation.y = 0.0;
  p.orientation.z = 0.0;
  p.orientation.w = 1.0;
  return p;
}

static bool plan_and_execute(moveit::planning_interface::MoveGroupInterface& mg, const geometry_msgs::msg::Pose& target, rclcpp::Logger logger)
{
  mg.setPoseTarget(target);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool ok = (mg.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

  if (!ok) {
    RCLCPP_WARN(logger, "Planning failed. Retrying once...");
    std::this_thread::sleep_for(300ms);
    ok = (mg.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
  }

  if (!ok) {
    RCLCPP_ERROR(logger, "Planning failed again.");
    return false;
  }

  auto exec_ok = (mg.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);
  if (!exec_ok) RCLCPP_WARN(logger, "Execute returned non-success.");
  return exec_ok;
}

static moveit_msgs::msg::CollisionObject make_box(
  const std::string& id,
  const std::string& frame_id,
  const geometry_msgs::msg::Pose& pose,
  double sx, double sy, double sz)
{
  moveit_msgs::msg::CollisionObject obj;
  obj.header.frame_id = frame_id;
  obj.id = id;

  shape_msgs::msg::SolidPrimitive prim;
  prim.type = prim.BOX;
  prim.dimensions = {sx, sy, sz};

  obj.primitives.push_back(prim);
  obj.primitive_poses.push_back(pose);
  obj.operation = moveit_msgs::msg::CollisionObject::ADD;
  return obj;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions opts;
  opts.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("hello_moveit_collision_cycle", opts);

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  std::thread spinner([&exec]() { exec.spin(); });

  static const std::string PLANNING_GROUP = "arm_torso";
  moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);
  move_group.setPlanningTime(5.0);
  move_group.setMaxVelocityScalingFactor(0.2);
  move_group.setMaxAccelerationScalingFactor(0.2);

  moveit::planning_interface::PlanningSceneInterface psi;

  const auto pos1 = make_pose(0.5,  0.3, 0.5);
  const auto pos2 = make_pose(0.5, -0.3, 0.5);

  auto box1_pose = make_pose(0.65, 0.0, 0.25);
  auto box1 = make_box("box1", "base_link", box1_pose, 0.4, 0.2, 0.1);

  auto box2_pose = make_pose(0.55, 0.0, 0.35);
  auto box2 = make_box("box2", "base_link", box2_pose, 0.2, 0.2, 0.2);

  std::string attach_link = "gripper_link";

  std::vector<std::string> touch_links = {
    attach_link
  };

  int iteration = 0;
  RCLCPP_INFO(node->get_logger(), "Starting iterations 1..6 (each iteration = one side-to-side move)");

  while (rclcpp::ok())
  {
    iteration++;
    int phase = ((iteration - 1) % 6) + 1;

    RCLCPP_INFO(node->get_logger(), "=== Iteration %d (Phase %d) ===", iteration, phase);

    if (phase == 1) {
      RCLCPP_INFO(node->get_logger(), "Phase1: ensuring scene is clear");
      psi.removeCollisionObjects(psi.getKnownObjectNames());
    }
    else if (phase == 2) {
      RCLCPP_INFO(node->get_logger(), "Phase2: adding box1 in front");
      psi.applyCollisionObject(box1);
    }
    else if (phase == 3) {
      RCLCPP_INFO(node->get_logger(), "Phase3: attaching box1 to link '%s'", attach_link.c_str());

      moveit_msgs::msg::AttachedCollisionObject aco;
      aco.link_name = attach_link;
      aco.object = box1;
      aco.touch_links = touch_links;

      psi.applyAttachedCollisionObject(aco);
    }
    else if (phase == 4) {
      RCLCPP_INFO(node->get_logger(), "Phase4: adding box2 in front");
      psi.applyCollisionObject(box2);
    }
    else if (phase == 5) {
      RCLCPP_INFO(node->get_logger(), "Phase5: detaching box1 from '%s'", attach_link.c_str());

      moveit_msgs::msg::AttachedCollisionObject detach;
      detach.link_name = attach_link;
      detach.object.id = "box1";
      detach.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;

      psi.applyAttachedCollisionObject(detach);
    }
    else if (phase == 6) {
      RCLCPP_INFO(node->get_logger(), "Phase6: clearing all collision objects");
      psi.removeCollisionObjects(psi.getKnownObjectNames());
    }

    std::this_thread::sleep_for(500ms);

    bool ok1 = plan_and_execute(move_group, pos1, node->get_logger());
    std::this_thread::sleep_for(500ms);
    bool ok2 = plan_and_execute(move_group, pos2, node->get_logger());

    if (!ok1 || !ok2) {
      RCLCPP_WARN(node->get_logger(), "One of the moves failed. Check planning group and link names.");
    }

    std::this_thread::sleep_for(1s);
  }

  exec.cancel();
  if (spinner.joinable()) spinner.join();
  rclcpp::shutdown();
  return 0;
}