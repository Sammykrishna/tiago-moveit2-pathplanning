#include "moveit_wrapper/moveit_wrapper.hpp"

#include <future>
#include <mutex>

namespace moveit_wrapper {

void MoveitWrapper::get_named_target_callback(
    const std::shared_ptr<
        moveit_wrapper_interfaces::srv::GetNamedTarget::Request>
        request,
    std::shared_ptr<moveit_wrapper_interfaces::srv::GetNamedTarget::Response>
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

    const auto &values = target_move_group->getNamedTargetValues(request->name);
    if (values.empty()) {
      response->success = false;
      response->message = "Named target '" + request->name +
                          "' not found in planning group '" +
                          target_move_group->getName() + "'";
      return;
    }

    sensor_msgs::msg::JointState js;
    for (const auto &[joint, position] : values) {
      js.name.push_back(joint);
      js.position.push_back(position);
    }

    response->joint_state = js;
    response->success = true;
    response->message = "Retrieved named target '" + request->name +
                        "' from group '" + target_move_group->getName() + "'";

  } catch (const std::exception &e) {
    response->success = false;
    response->message = "Exception: " + std::string(e.what());
    RCLCPP_ERROR(this->get_logger(), "GetNamedTarget error: %s", e.what());
  }
}

} // namespace moveit_wrapper
