#include "moveit_wrapper/moveit_wrapper.hpp"

#include <mutex>
#include <string>
#include <vector>

namespace moveit_wrapper {

void MoveitWrapper::attach_object_callback(
    const std::shared_ptr<moveit_wrapper_interfaces::srv::AttachObj::Request>
        request,
    std::shared_ptr<moveit_wrapper_interfaces::srv::AttachObj::Response>
        response) {

  if (!is_moveit_ready()) {
    response->success = false;
    response->message = "MoveIt not ready yet";
    return;
  }

  if (request->move_group.empty()) {
    response->success = false;
    response->message = "move_group must be specified to attach objects";
    return;
  }

  if (request->object_names.empty()) {
    response->success = false;
    response->message = "object_names must not be empty";
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

    for (const std::string &object_name : request->object_names) {
      const bool ok = target_move_group->attachObject(
          object_name, std::string(""), gripper_touch_links_);

      if (!ok) {
        RCLCPP_WARN(this->get_logger(),
                    "Failed to attach object '%s' via MoveGroupInterface",
                    object_name.c_str());
        continue;
      }

      planning_scene_interface_->removeCollisionObjects({object_name});
    }

    response->success = true;
    response->message =
        "Attached " + std::to_string(request->object_names.size()) +
        " objects (using gripper_touch_links) and removed them from world";

  } catch (const std::exception &e) {
    response->success = false;
    response->message = "Failed to attach objects: " + std::string(e.what());
    RCLCPP_ERROR(this->get_logger(), "Attach object error: %s", e.what());
  }
}

} // namespace moveit_wrapper
