#include "moveit_wrapper/moveit_wrapper.hpp"

#include <future>
#include <mutex>
#include <vector>

namespace moveit_wrapper {

void MoveitWrapper::add_collision_object_callback(
    const std::shared_ptr<moveit_wrapper_interfaces::srv::AddColObj::Request>
        request,
    std::shared_ptr<moveit_wrapper_interfaces::srv::AddColObj::Response>
        response) {

  if (!is_moveit_ready()) {
    response->success = false;
    response->message = "MoveIt not ready yet";
    return;
  }

  try {
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    collision_objects.reserve(request->objects.size());

    for (const auto &marker : request->objects) {
      auto obj_opt =
          marker_to_collision_object_safe(marker, this->get_logger());
      if (!obj_opt) {
        continue;
      }
      collision_objects.push_back(*obj_opt);
    }

    if (collision_objects.empty()) {
      response->success = false;
      response->message =
          "No valid collision objects to add (all markers invalid or skipped)";
      RCLCPP_WARN(this->get_logger(),
                  "add_collision_object_callback: no valid objects added");
      return;
    }

    planning_scene_interface_->addCollisionObjects(collision_objects);
    response->success = true;
    response->message = "Added " + std::to_string(collision_objects.size()) +
                        " collision objects";

  } catch (const std::exception &e) {
    response->success = false;
    response->message =
        "Failed to add collision objects: " + std::string(e.what());
    RCLCPP_ERROR(this->get_logger(), "Add collision object error: %s",
                 e.what());
  }
}

} // namespace moveit_wrapper
