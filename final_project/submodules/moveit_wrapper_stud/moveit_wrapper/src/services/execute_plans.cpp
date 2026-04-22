#include "moveit_wrapper/moveit_wrapper.hpp"

#include <mutex>

namespace moveit_wrapper {

void MoveitWrapper::execute_plans_callback(
    const std::shared_ptr<moveit_wrapper_interfaces::srv::ExecutePlans::Request>
        request,
    std::shared_ptr<moveit_wrapper_interfaces::srv::ExecutePlans::Response>
        response) {
  if (!is_moveit_ready()) {
    response->success = false;
    response->message = "MoveIt not ready yet";
    return;
  }

  if (request->move_group.empty()) {
    response->success = false;
    response->message = "move_group must be specified for execution";
    return;
  }

  std::lock_guard<std::mutex> lock(move_group_mutex_);

  auto execution_move_group = get_move_group(request->move_group);
  if (!execution_move_group) {
    response->success = false;
    response->message = "Execution planning group '" + request->move_group +
                        "' not found. Available: ";
    for (const auto &entry : move_groups_) {
      response->message += entry.first + " ";
    }
    return;
  }

  try {
    bool all_success = true;
    std::string error_message;

    for (size_t i = 0; i < request->plans.size(); ++i) {
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      plan.trajectory_ = request->plans[i];

      auto result = execution_move_group->execute(plan);

      if (result != moveit::core::MoveItErrorCode::SUCCESS) {
        all_success = false;
        error_message = "Failed to execute plan " + std::to_string(i);
        break;
      }
    }

    response->success = all_success;
    response->message = all_success
                            ? ("All " + std::to_string(request->plans.size()) +
                               " plans executed successfully using group '" +
                               execution_move_group->getName() + "'")
                            : error_message;
  } catch (const std::exception &e) {
    response->success = false;
    response->message = "Execution error: " + std::string(e.what());
    RCLCPP_ERROR(this->get_logger(), "Execution error: %s", e.what());
  }
}

} // namespace moveit_wrapper
