#include "moveit_wrapper/moveit_wrapper.hpp"

#include <future>
#include <mutex>

namespace moveit_wrapper {

void MoveitWrapper::get_named_targets_callback(
    const std::shared_ptr<
        moveit_wrapper_interfaces::srv::GetNamedTargets::Request>
        request,
    std::shared_ptr<moveit_wrapper_interfaces::srv::GetNamedTargets::Response>
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

    const auto &targets = target_move_group->getNamedTargets();
    response->names.assign(targets.begin(), targets.end());
    response->success = true;
    response->message = "Retrieved " + std::to_string(targets.size()) +
                        " named targets for group '" +
                        target_move_group->getName() + "'";

  } catch (const std::exception &e) {
    response->success = false;
    response->message = "Exception: " + std::string(e.what());
    RCLCPP_ERROR(this->get_logger(), "GetNamedTargets error: %s", e.what());
  }
}

} // namespace moveit_wrapper
