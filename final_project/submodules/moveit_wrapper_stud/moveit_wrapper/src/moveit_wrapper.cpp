#include "moveit_wrapper/moveit_wrapper.hpp"

#include <set>

namespace moveit_wrapper {

MoveitWrapper::MoveitWrapper(const rclcpp::NodeOptions &options)
    : Node("moveit_wrapper", [](rclcpp::NodeOptions opts) {
        opts.automatically_declare_parameters_from_overrides(true);
        return opts;
      }(options)) {

  load_parameters();

  param_callback_handle_ = this->add_on_set_parameters_callback(std::bind(
      &MoveitWrapper::on_parameters_changed, this, std::placeholders::_1));

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  init_services();

  RCLCPP_INFO(this->get_logger(),
              "MoveitWrapper initialized. Waiting for MoveIt ...");

  init_timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&MoveitWrapper::initialize_moveit, this));
}

MoveitWrapper::~MoveitWrapper() = default;

void MoveitWrapper::load_parameters() {
  this->get_parameter("base_frame", base_frame_);
  this->get_parameter("ee_frame", ee_frame_);
  this->get_parameter("tf_timeout", tf_timeout_);
  this->get_parameter("execute_timeout", execute_timeout_);
  this->get_parameter("max_torso_joint_value", max_torso_joint_value_);
  this->get_parameter("torso_joint_name", torso_joint_name_);

  if (!this->get_parameter("gripper_touch_links", gripper_touch_links_)) {
    gripper_touch_links_.clear();
  }

  group_configs_.clear();

  if (!this->get_parameter("planning_groups", planing_groups_) ||
      planing_groups_.empty()) {
    RCLCPP_ERROR(this->get_logger(), "No planning_groups defined!");
    return;
  }

  for (const auto &group_name : planing_groups_) {
    PlanningGroupConfig config;

    this->get_parameter(group_name + ".planning_time", config.planning_time);
    this->get_parameter(group_name + ".max_velocity_scaling_factor",
                        config.max_velocity_scaling_factor);
    this->get_parameter(group_name + ".max_acceleration_scaling_factor",
                        config.max_acceleration_scaling_factor);
    this->get_parameter(group_name + ".planning_attempts",
                        config.planning_attempts);
    this->get_parameter(group_name + ".planner_id", config.planner_id);
    this->get_parameter(group_name + ".planning_pipeline_id",
                        config.planning_pipeline_id);

    group_configs_[group_name] = config;

    RCLCPP_INFO(this->get_logger(),
                "Loaded config for group '%s': time=%.1fs vel=%.2f acc=%.2f "
                "attempts=%d",
                group_name.c_str(), config.planning_time,
                config.max_velocity_scaling_factor,
                config.max_acceleration_scaling_factor,
                config.planning_attempts);
  }
  if (!gripper_touch_links_.empty()) {
    std::string s;
    for (const auto &l : gripper_touch_links_) {
      s += l + " ";
    }
    RCLCPP_INFO(this->get_logger(), "gripper_touch_links: %s", s.c_str());
  } else {
    RCLCPP_WARN(
        this->get_logger(),
        "gripper_touch_links empty -> MoveIt will default to link_name");
  }
}

rcl_interfaces::msg::SetParametersResult MoveitWrapper::on_parameters_changed(
    const std::vector<rclcpp::Parameter> &params) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "";

  try {
    std::set<std::string> groups_to_update;

    for (const auto &param : params) {
      if (param.get_name() == "base_frame") {
        base_frame_ = param.as_string();
        result.reason += "base_frame updated; ";
      } else if (param.get_name() == "ee_frame") {
        ee_frame_ = param.as_string();
        result.reason += "ee_frame updated; ";
      } else if (param.get_name() == "tf_timeout") {
        tf_timeout_ = param.as_double();
        result.reason += "tf_timeout updated; ";
      } else {
        for (const auto &group_name : planing_groups_) {
          std::string group_prefix = group_name + "/";
          if (param.get_name().find(group_prefix) == 0) {
            std::string param_name =
                param.get_name().substr(group_prefix.length());

            if (group_configs_.find(group_name) != group_configs_.end()) {
              auto &config = group_configs_[group_name];

              if (param_name == "planning_time") {
                config.planning_time = param.as_double();
              } else if (param_name == "max_velocity_scaling_factor") {
                config.max_velocity_scaling_factor = param.as_double();
              } else if (param_name == "max_acceleration_scaling_factor") {
                config.max_acceleration_scaling_factor = param.as_double();
              } else if (param_name == "planning_attempts") {
                config.planning_attempts = param.as_int();
              } else if (param_name == "planner_id") {
                config.planner_id = param.as_string();
              } else if (param_name == "planning_pipeline_id") {
                config.planning_pipeline_id = param.as_string();
              }

              groups_to_update.insert(group_name);
            }

            result.reason += group_name + "/" + param_name + " updated; ";
            break;
          }
        }
      }
    }

    if (moveit_ready_ && !groups_to_update.empty()) {
      std::lock_guard<std::mutex> lock(move_group_mutex_);
      for (const std::string &group_name : groups_to_update) {
        auto it = move_groups_.find(group_name);
        if (it != move_groups_.end()) {
          configure_move_group(group_name, it->second);
          RCLCPP_INFO(this->get_logger(),
                      "Updated configuration for planning group: %s",
                      group_name.c_str());
        }
      }
    }

  } catch (const std::exception &e) {
    result.successful = false;
    result.reason = std::string("Exception: ") + e.what();
    RCLCPP_ERROR(this->get_logger(), "on_parameters_changed failed: %s",
                 e.what());
  } catch (...) {
    result.successful = false;
    result.reason = "Unknown exception in on_parameters_changed";
    RCLCPP_ERROR(this->get_logger(),
                 "on_parameters_changed failed: Unknown exception");
  }

  return result;
}

void MoveitWrapper::init_services() {
  plan_service_ = this->create_service<moveit_wrapper_interfaces::srv::Plan>(
      "~/plan", std::bind(&MoveitWrapper::plan_callback, this,
                          std::placeholders::_1, std::placeholders::_2));

  plan_joints_service_ =
      this->create_service<moveit_wrapper_interfaces::srv::PlanJoints>(
          "~/plan_joints",
          std::bind(&MoveitWrapper::plan_joints_callback, this,
                    std::placeholders::_1, std::placeholders::_2));

  execute_plans_service_ =
      this->create_service<moveit_wrapper_interfaces::srv::ExecutePlans>(
          "~/execute_plans",
          std::bind(&MoveitWrapper::execute_plans_callback, this,
                    std::placeholders::_1, std::placeholders::_2));

  add_collision_object_service_ =
      this->create_service<moveit_wrapper_interfaces::srv::AddColObj>(
          "~/add_collision_object",
          std::bind(&MoveitWrapper::add_collision_object_callback, this,
                    std::placeholders::_1, std::placeholders::_2));

  remove_collision_object_service_ =
      this->create_service<moveit_wrapper_interfaces::srv::RemoveColObj>(
          "~/remove_collision_object",
          std::bind(&MoveitWrapper::remove_collision_object_callback, this,
                    std::placeholders::_1, std::placeholders::_2));

  get_collision_objects_service_ =
      this->create_service<moveit_wrapper_interfaces::srv::GetColObj>(
          "~/get_collision_objects",
          std::bind(&MoveitWrapper::get_collision_objects_callback, this,
                    std::placeholders::_1, std::placeholders::_2));

  attach_object_service_ =
      this->create_service<moveit_wrapper_interfaces::srv::AttachObj>(
          "~/attach_object",
          std::bind(&MoveitWrapper::attach_object_callback, this,
                    std::placeholders::_1, std::placeholders::_2));

  detach_object_service_ =
      this->create_service<moveit_wrapper_interfaces::srv::DettachObj>(
          "~/detach_object",
          std::bind(&MoveitWrapper::detach_object_callback, this,
                    std::placeholders::_1, std::placeholders::_2));

  get_attached_objects_service_ =
      this->create_service<moveit_wrapper_interfaces::srv::GetAttachedObjs>(
          "~/get_attached_objects",
          std::bind(&MoveitWrapper::get_attached_objects_callback, this,
                    std::placeholders::_1, std::placeholders::_2));

  get_named_target_service_ =
      this->create_service<moveit_wrapper_interfaces::srv::GetNamedTarget>(
          "~/get_named_target",
          std::bind(&MoveitWrapper::get_named_target_callback, this,
                    std::placeholders::_1, std::placeholders::_2));

  get_named_targets_service_ =
      this->create_service<moveit_wrapper_interfaces::srv::GetNamedTargets>(
          "~/get_named_targets",
          std::bind(&MoveitWrapper::get_named_targets_callback, this,
                    std::placeholders::_1, std::placeholders::_2));

  goto_named_target_service_ =
      this->create_service<moveit_wrapper_interfaces::srv::GotoNamedTarget>(
          "~/goto_named_target",
          std::bind(&MoveitWrapper::goto_named_target_callback, this,
                    std::placeholders::_1, std::placeholders::_2));

  if (max_torso_joint_value_ != 0.0 && !torso_joint_name_.empty()) {
    move_torso_service_ =
        this->create_service<moveit_wrapper_interfaces::srv::MoveTorso>(
            "~/move_torso",
            std::bind(&MoveitWrapper::move_torso_callback, this,
                      std::placeholders::_1, std::placeholders::_2));
  }

  RCLCPP_INFO(this->get_logger(), "All services initialized");
}

void MoveitWrapper::initialize_moveit() {
  if (moveit_ready_) {
    init_timer_->cancel();
    return;
  }

  try {
    RCLCPP_INFO(this->get_logger(), "Initializing MoveIt interfaces...");
    auto node = std::static_pointer_cast<rclcpp::Node>(shared_from_this());

    for (const auto &[group_name, config] : group_configs_) {
      (void)config;
      try {
        RCLCPP_INFO(this->get_logger(), "Initializing move group: %s",
                    group_name.c_str());

        auto move_group =
            std::make_shared<moveit::planning_interface::MoveGroupInterface>(
                node, group_name);

        configure_move_group(group_name, move_group);
        move_groups_[group_name] = move_group;

        RCLCPP_INFO(this->get_logger(),
                    "Successfully initialized move group: %s",
                    group_name.c_str());

      } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(),
                     "Failed to initialize move group '%s': %s",
                     group_name.c_str(), e.what());
      }
    }

    if (move_groups_.empty()) {
      RCLCPP_ERROR(this->get_logger(),
                   "No move groups successfully initialized!");
      moveit_ready_ = false;
      return;
    }

    planning_scene_interface_ =
        std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

    RCLCPP_INFO(this->get_logger(), "MoveIt wrapper successfully initialized:");
    RCLCPP_INFO(this->get_logger(), "  Initialized groups:");

    for (const auto &[group_name, group] : move_groups_) {
      RCLCPP_INFO(this->get_logger(), "    - %s (joints: %zu)",
                  group_name.c_str(), group->getJointNames().size());
    }

    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> ref_group;

    if (!planing_groups_.empty()) {
      auto it = move_groups_.find(planing_groups_.front());
      if (it != move_groups_.end()) {
        ref_group = it->second;
      }
    }

    if (!ref_group) {
      ref_group = move_groups_.begin()->second;
    }

    if (ref_group) {
      RCLCPP_INFO(this->get_logger(), "  Reference planning group: %s",
                  ref_group->getName().c_str());
      RCLCPP_INFO(this->get_logger(), "  End effector link: %s",
                  ref_group->getEndEffectorLink().c_str());
      RCLCPP_INFO(this->get_logger(), "  Planning frame: %s",
                  ref_group->getPlanningFrame().c_str());
    }

    RCLCPP_INFO(this->get_logger(), "  Base frame: %s", base_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "  EE frame: %s", ee_frame_.c_str());
    if (torso_joint_name_.empty()) {
      RCLCPP_INFO(this->get_logger(), "  Torso joint: N/A");
    } else {
      RCLCPP_INFO(this->get_logger(), "  Torso joint: %s",
                  torso_joint_name_.c_str());
      RCLCPP_INFO(this->get_logger(), "  Max torso joint value: %f",
                  max_torso_joint_value_);
    }

    moveit_ready_ = true;
    init_timer_->cancel();

    RCLCPP_INFO(this->get_logger(),
                "MoveitWrapper is ready to receive requests!");

  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize MoveIt: %s",
                 e.what());
    moveit_ready_ = false;
  }
}

void MoveitWrapper::configure_move_group(
    const std::string &group_name,
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface>
        move_group) {
  const auto &config = group_configs_.at(group_name);
  move_group->setEndEffectorLink(ee_frame_);
  move_group->setPlanningTime(config.planning_time);
  move_group->setMaxVelocityScalingFactor(config.max_velocity_scaling_factor);
  move_group->setMaxAccelerationScalingFactor(
      config.max_acceleration_scaling_factor);
  move_group->setNumPlanningAttempts(config.planning_attempts);

  if (!config.planner_id.empty()) {
    move_group->setPlannerId(config.planner_id);
  }
  if (!config.planning_pipeline_id.empty()) {
    move_group->setPlanningPipelineId(config.planning_pipeline_id);
  }
}

std::shared_ptr<moveit::planning_interface::MoveGroupInterface>
MoveitWrapper::get_move_group(const std::string &group_name) {
  auto it = move_groups_.find(group_name);
  if (it != move_groups_.end()) {
    return it->second;
  }
  return nullptr;
}

bool MoveitWrapper::is_moveit_ready() { return moveit_ready_; }

geometry_msgs::msg::PoseStamped
MoveitWrapper::transform_pose(const geometry_msgs::msg::PoseStamped &pose,
                              const std::string &target_frame) {
  geometry_msgs::msg::PoseStamped out;
  try {
    out = tf_buffer_->transform(pose, target_frame,
                                tf2::durationFromSec(tf_timeout_));
  } catch (const tf2::TransformException &ex) {
    RCLCPP_WARN(this->get_logger(), "Transform failed: %s", ex.what());
    out = pose;
  }
  return out;
}

void MoveitWrapper::apply_path_constraints(
    moveit::planning_interface::MoveGroupInterface &move_group,
    const moveit_msgs::msg::Constraints &global_constraints,
    const moveit_msgs::msg::Constraints &local_constraints) {
  move_group.clearPathConstraints();

  moveit_msgs::msg::Constraints merged;
  merged.name = "merged_constraints";

  auto append_constraints = [](const moveit_msgs::msg::Constraints &src,
                               moveit_msgs::msg::Constraints &dst) {
    dst.position_constraints.insert(dst.position_constraints.end(),
                                    src.position_constraints.begin(),
                                    src.position_constraints.end());
    dst.orientation_constraints.insert(dst.orientation_constraints.end(),
                                       src.orientation_constraints.begin(),
                                       src.orientation_constraints.end());
    dst.joint_constraints.insert(dst.joint_constraints.end(),
                                 src.joint_constraints.begin(),
                                 src.joint_constraints.end());
    dst.visibility_constraints.insert(dst.visibility_constraints.end(),
                                      src.visibility_constraints.begin(),
                                      src.visibility_constraints.end());
  };

  append_constraints(global_constraints, merged);
  append_constraints(local_constraints, merged);

  RCLCPP_INFO(
      this->get_logger(),
      "Setting %zu position, %zu orientation, %zu joint, %zu visibility "
      "constraints",
      merged.position_constraints.size(), merged.orientation_constraints.size(),
      merged.joint_constraints.size(), merged.visibility_constraints.size());
  move_group.setPathConstraints(merged);
}

} // namespace moveit_wrapper

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(moveit_wrapper::MoveitWrapper)
