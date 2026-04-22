#pragma once

#include <mutex>
#include <rclcpp/rclcpp.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/conversions.h>

#include <std_msgs/msg/string.hpp>

#include "moveit_wrapper_interfaces/msg/waypoint.hpp"

#include "moveit_wrapper_interfaces/srv/add_col_obj.hpp"
#include "moveit_wrapper_interfaces/srv/attach_obj.hpp"
#include "moveit_wrapper_interfaces/srv/dettach_obj.hpp"
#include "moveit_wrapper_interfaces/srv/execute_plans.hpp"
#include "moveit_wrapper_interfaces/srv/get_attached_objs.hpp"
#include "moveit_wrapper_interfaces/srv/get_col_obj.hpp"
#include "moveit_wrapper_interfaces/srv/get_named_target.hpp"
#include "moveit_wrapper_interfaces/srv/get_named_targets.hpp"
#include "moveit_wrapper_interfaces/srv/goto_named_target.hpp"
#include "moveit_wrapper_interfaces/srv/move_torso.hpp"
#include "moveit_wrapper_interfaces/srv/plan.hpp"
#include "moveit_wrapper_interfaces/srv/plan_joints.hpp"
#include "moveit_wrapper_interfaces/srv/remove_col_obj.hpp"

#include "moveit_wrapper/marker_utils.hpp"

namespace moveit_wrapper {

class MoveitWrapper : public rclcpp::Node {
public:
  explicit MoveitWrapper(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~MoveitWrapper();

private:
  void load_parameters();
  void init_services();
  void initialize_moveit();
  bool is_moveit_ready();

  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
  rcl_interfaces::msg::SetParametersResult
  on_parameters_changed(const std::vector<rclcpp::Parameter> &params);

  void plan_callback(
      const std::shared_ptr<moveit_wrapper_interfaces::srv::Plan::Request>
          request,
      std::shared_ptr<moveit_wrapper_interfaces::srv::Plan::Response> response);

  void plan_joints_callback(
      const std::shared_ptr<moveit_wrapper_interfaces::srv::PlanJoints::Request>
          request,
      std::shared_ptr<moveit_wrapper_interfaces::srv::PlanJoints::Response>
          response);

  void execute_plans_callback(
      const std::shared_ptr<
          moveit_wrapper_interfaces::srv::ExecutePlans::Request>
          request,
      std::shared_ptr<moveit_wrapper_interfaces::srv::ExecutePlans::Response>
          response);

  void add_collision_object_callback(
      const std::shared_ptr<moveit_wrapper_interfaces::srv::AddColObj::Request>
          request,
      std::shared_ptr<moveit_wrapper_interfaces::srv::AddColObj::Response>
          response);

  void remove_collision_object_callback(
      const std::shared_ptr<
          moveit_wrapper_interfaces::srv::RemoveColObj::Request>
          request,
      std::shared_ptr<moveit_wrapper_interfaces::srv::RemoveColObj::Response>
          response);

  void get_collision_objects_callback(
      const std::shared_ptr<moveit_wrapper_interfaces::srv::GetColObj::Request>
          request,
      std::shared_ptr<moveit_wrapper_interfaces::srv::GetColObj::Response>
          response);

  void attach_object_callback(
      const std::shared_ptr<moveit_wrapper_interfaces::srv::AttachObj::Request>
          request,
      std::shared_ptr<moveit_wrapper_interfaces::srv::AttachObj::Response>
          response);

  void detach_object_callback(
      const std::shared_ptr<moveit_wrapper_interfaces::srv::DettachObj::Request>
          request,
      std::shared_ptr<moveit_wrapper_interfaces::srv::DettachObj::Response>
          response);

  void get_attached_objects_callback(
      const std::shared_ptr<
          moveit_wrapper_interfaces::srv::GetAttachedObjs::Request>
          request,
      std::shared_ptr<moveit_wrapper_interfaces::srv::GetAttachedObjs::Response>
          response);

  void get_named_target_callback(
      const std::shared_ptr<
          moveit_wrapper_interfaces::srv::GetNamedTarget::Request>
          request,
      std::shared_ptr<moveit_wrapper_interfaces::srv::GetNamedTarget::Response>
          response);

  void get_named_targets_callback(
      const std::shared_ptr<
          moveit_wrapper_interfaces::srv::GetNamedTargets::Request>
          request,
      std::shared_ptr<moveit_wrapper_interfaces::srv::GetNamedTargets::Response>
          response);

  void goto_named_target_callback(
      const std::shared_ptr<
          moveit_wrapper_interfaces::srv::GotoNamedTarget::Request>
          request,
      std::shared_ptr<moveit_wrapper_interfaces::srv::GotoNamedTarget::Response>
          response);

  void move_torso_callback(
      const std::shared_ptr<moveit_wrapper_interfaces::srv::MoveTorso::Request>
          request,
      std::shared_ptr<moveit_wrapper_interfaces::srv::MoveTorso::Response>
          response);

  geometry_msgs::msg::PoseStamped
  transform_pose(const geometry_msgs::msg::PoseStamped &pose,
                 const std::string &target_frame);
  void apply_path_constraints(
      moveit::planning_interface::MoveGroupInterface &move_group,
      const moveit_msgs::msg::Constraints &global_constraints,
      const moveit_msgs::msg::Constraints &local_constraints);
  void configure_move_group(
      const std::string &group_name,
      std::shared_ptr<moveit::planning_interface::MoveGroupInterface>
          move_group);
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface>
  get_move_group(const std::string &group_name);

  struct PlanningGroupConfig {
    double planning_time;
    double max_velocity_scaling_factor;
    double max_acceleration_scaling_factor;
    int planning_attempts;
    std::string planner_id;
    std::string planning_pipeline_id;
  };
  std::map<std::string, PlanningGroupConfig> group_configs_;
  std::map<std::string,
           std::shared_ptr<moveit::planning_interface::MoveGroupInterface>>
      move_groups_;

  std::string base_frame_;
  std::string ee_frame_;
  double tf_timeout_;
  double execute_timeout_;
  double max_torso_joint_value_;
  std::string torso_joint_name_;
  std::vector<std::string> gripper_touch_links_; // links to disable collision
                                                 // when attaching an object
  std::vector<std::string> planing_groups_;

  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface>
      planning_scene_interface_;
  std::shared_ptr<moveit_cpp::MoveItCpp> moveit_cpp_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Service<moveit_wrapper_interfaces::srv::Plan>::SharedPtr
      plan_service_;
  rclcpp::Service<moveit_wrapper_interfaces::srv::PlanJoints>::SharedPtr
      plan_joints_service_;
  rclcpp::Service<moveit_wrapper_interfaces::srv::ExecutePlans>::SharedPtr
      execute_plans_service_;
  rclcpp::Service<moveit_wrapper_interfaces::srv::AddColObj>::SharedPtr
      add_collision_object_service_;
  rclcpp::Service<moveit_wrapper_interfaces::srv::RemoveColObj>::SharedPtr
      remove_collision_object_service_;
  rclcpp::Service<moveit_wrapper_interfaces::srv::GetColObj>::SharedPtr
      get_collision_objects_service_;
  rclcpp::Service<moveit_wrapper_interfaces::srv::AttachObj>::SharedPtr
      attach_object_service_;
  rclcpp::Service<moveit_wrapper_interfaces::srv::DettachObj>::SharedPtr
      detach_object_service_;
  rclcpp::Service<moveit_wrapper_interfaces::srv::GetAttachedObjs>::SharedPtr
      get_attached_objects_service_;
  rclcpp::Service<moveit_wrapper_interfaces::srv::GetNamedTarget>::SharedPtr
      get_named_target_service_;
  rclcpp::Service<moveit_wrapper_interfaces::srv::GetNamedTargets>::SharedPtr
      get_named_targets_service_;
  rclcpp::Service<moveit_wrapper_interfaces::srv::GotoNamedTarget>::SharedPtr
      goto_named_target_service_;
  rclcpp::Service<moveit_wrapper_interfaces::srv::MoveTorso>::SharedPtr
      move_torso_service_;

  std::mutex move_group_mutex_;
  std::mutex plans_mutex_;
  std::vector<moveit::planning_interface::MoveGroupInterface::Plan>
      stored_plans_;
  std::atomic<bool> moveit_ready_{false};
  rclcpp::TimerBase::SharedPtr init_timer_;
};

} // namespace moveit_wrapper
