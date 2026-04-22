#include "moveit_wrapper/moveit_wrapper.hpp"

#include <mutex>

namespace moveit_wrapper {

void MoveitWrapper::goto_named_target_callback(
    const std::shared_ptr<
        moveit_wrapper_interfaces::srv::GotoNamedTarget::Request>
        request,
    std::shared_ptr<moveit_wrapper_interfaces::srv::GotoNamedTarget::Response>
        response) {
  if (!is_moveit_ready()) {
    response->success = false;
    response->message = "MoveIt not ready yet";
    return;
  }

  if (request->move_group.empty()) {
    response->success = false;
    response->message = "move_group must be specified";
    return;
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

    std::string group_name = target_move_group->getName();
    RCLCPP_INFO(this->get_logger(), "Using planning group: %s for target: %s",
                group_name.c_str(), request->name.c_str());

    target_move_group->setStartStateToCurrentState();

    const auto &available_targets = target_move_group->getNamedTargets();
    if (std::find(available_targets.begin(), available_targets.end(),
                  request->name) == available_targets.end()) {
      response->success = false;
      response->message = "Named target '" + request->name +
                          "' not found in group '" + group_name +
                          "'. Available: ";
      for (const auto &target : available_targets) {
        response->message += target + " ";
      }
      return;
    }

    const auto &joint_values =
        target_move_group->getNamedTargetValues(request->name);
    target_move_group->setJointValueTarget(joint_values);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool plan_success = (target_move_group->plan(plan) ==
                         moveit::core::MoveItErrorCode::SUCCESS);

    if (!plan_success) {
      response->success = false;
      response->message = "Planning failed for " + group_name + " target '" +
                          request->name + "'";
      return;
    }

    auto exec_result = target_move_group->execute(plan);
    if (exec_result == moveit::core::MoveItErrorCode::SUCCESS) {
      response->success = true;
      response->message = "Successfully moved " + group_name + " to target '" +
                          request->name + "'";
    } else {
      response->success = false;
      response->message = "Execution failed for " + group_name + " target '" +
                          request->name + "'";
    }
  } catch (const std::exception &e) {
    response->success = false;
    response->message = "Exception: " + std::string(e.what());
    RCLCPP_ERROR(this->get_logger(), "GotoNamedTarget error: %s", e.what());
  }
}

} // namespace moveit_wrapper
