#include "moveit_wrapper/moveit_wrapper.hpp"

#include <future>
#include <mutex>

namespace moveit_wrapper {

void MoveitWrapper::get_collision_objects_callback(
    const std::shared_ptr<
        moveit_wrapper_interfaces::srv::GetColObj::Request> /* request */,
    std::shared_ptr<moveit_wrapper_interfaces::srv::GetColObj::Response>
        response) {

  if (!is_moveit_ready()) {
    response->success = false;
    response->message = "MoveIt not ready yet";
    return;
  }

  try {
    auto objects = planning_scene_interface_->getObjects();

    // Convert CollisionObjects to visualization_msgs::Marker
    for (const auto &[name, obj] : objects) {
      response->objects.push_back(collision_object_to_marker(obj, name));
    }

    response->success = true;
    response->message =
        "Retrieved " + std::to_string(objects.size()) + " collision objects";

  } catch (const std::exception &e) {
    response->success = false;
    response->message =
        "Failed to get collision objects: " + std::string(e.what());
    RCLCPP_ERROR(this->get_logger(), "Get collision objects error: %s",
                 e.what());
  }
}

} // namespace moveit_wrapper
