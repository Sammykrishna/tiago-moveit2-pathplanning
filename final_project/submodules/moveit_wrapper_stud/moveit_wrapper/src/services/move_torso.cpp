#include "moveit_wrapper/moveit_wrapper.hpp"

#include <future>
#include <mutex>

namespace moveit_wrapper {

void MoveitWrapper::move_torso_callback(
    const std::shared_ptr<moveit_wrapper_interfaces::srv::MoveTorso::Request>
        request,
    std::shared_ptr<moveit_wrapper_interfaces::srv::MoveTorso::Response>
        response) {
  if (!is_moveit_ready()) {
    response->success = false;
    response->message = "MoveIt is not ready";
    return;
  }

  if (request->move_group.empty()) {
    response->success = false;
    response->message = "move_group must be specified";
    return;
  }

  double wanted_value = request->joint_value;
  if (wanted_value > max_torso_joint_value_) {
    wanted_value = max_torso_joint_value_;
  }

  std::lock_guard<std::mutex> lock(move_group_mutex_);

  try {
    auto target_move_group = get_move_group(request->move_group);

    if (!target_move_group) {
      response->success = false;
      response->message =
          "Planning group '" + request->move_group + "' not found. Available: ";
      for (const auto &[name, group] : move_groups_) {
        (void)group;
        response->message += name + " ";
      }
      return;
    }

    RCLCPP_INFO(this->get_logger(),
                "Getting current joint values for move group: %s",
                target_move_group->getName().c_str());

    moveit::core::RobotStatePtr current_state =
        target_move_group->getCurrentState(2.0);

    if (!current_state) {
      response->success = false;
      response->message = "Timed out waiting for current robot state";
      RCLCPP_ERROR(this->get_logger(),
                   "Timeout waiting for current robot state");
      return;
    }

    const std::vector<std::string> &joint_names =
        target_move_group->getJointNames();

    std::map<std::string, double> joint_targets;

    for (const auto &jn : joint_names) {
      joint_targets[jn] = current_state->getJointPositions(jn)[0];
    }

    if (joint_targets.find(torso_joint_name_) == joint_targets.end()) {
      response->success = false;
      response->message = "Torso joint '" + torso_joint_name_ +
                          "' not part of planning group '" +
                          target_move_group->getName() + "'";
      return;
    }

    double new_value = wanted_value;

    if (request->relative) {
      new_value = joint_targets[torso_joint_name_] + wanted_value;
      if (new_value > max_torso_joint_value_) {
        new_value = max_torso_joint_value_;
      }
    }

    joint_targets[torso_joint_name_] = new_value;

    target_move_group->setJointValueTarget(joint_targets);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (target_move_group->plan(plan) ==
                    moveit::core::MoveItErrorCode::SUCCESS);

    if (!success) {
      response->success = false;
      response->message = "Planning torso movement failed using group '" +
                          target_move_group->getName() + "'";
      return;
    }

    {
      std::lock_guard<std::mutex> plans_lock(plans_mutex_);
      stored_plans_.push_back(plan);
    }

    response->success = true;
    response->message = "Torso movement planned successfully using group '" +
                        target_move_group->getName() + "'";

  } catch (const std::exception &e) {
    response->success = false;
    response->message = "Exception: " + std::string(e.what());
    RCLCPP_ERROR(this->get_logger(), "move_torso_callback error: %s", e.what());
  }
}

} // namespace moveit_wrapper
