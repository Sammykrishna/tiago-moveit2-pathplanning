#include "moveit_wrapper/moveit_wrapper.hpp"

#include <future>
#include <mutex>

namespace moveit_wrapper {

void MoveitWrapper::get_attached_objects_callback(
    const std::shared_ptr<
        moveit_wrapper_interfaces::srv::GetAttachedObjs::Request> /* request */,
    std::shared_ptr<moveit_wrapper_interfaces::srv::GetAttachedObjs::Response>
        response) {

  if (!is_moveit_ready()) {
    response->success = false;
    response->message = "MoveIt not ready yet";
    return;
  }

  try {
    auto attached_objects = planning_scene_interface_->getAttachedObjects();

    // Convert AttachedCollisionObjects to visualization_msgs::Marker
    for (const auto &[name, attached_obj] : attached_objects) {
      response->objects.push_back(
          attached_collision_object_to_marker(attached_obj, name));
    }

    response->success = true;
    response->message = "Retrieved " + std::to_string(attached_objects.size()) +
                        " attached objects";

  } catch (const std::exception &e) {
    response->success = false;
    response->message =
        "Failed to get attached objects: " + std::string(e.what());
    RCLCPP_ERROR(this->get_logger(), "Get attached objects error: %s",
                 e.what());
  }
}

} // namespace moveit_wrapper
