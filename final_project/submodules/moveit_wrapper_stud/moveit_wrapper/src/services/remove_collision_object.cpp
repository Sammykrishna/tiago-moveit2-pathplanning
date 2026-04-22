#include "moveit_wrapper/moveit_wrapper.hpp"

#include <future>
#include <mutex>
#include <string>
#include <vector>

namespace moveit_wrapper {

void MoveitWrapper::remove_collision_object_callback(
    const std::shared_ptr<moveit_wrapper_interfaces::srv::RemoveColObj::Request>
        request,
    std::shared_ptr<moveit_wrapper_interfaces::srv::RemoveColObj::Response>
        response) {

  if (!is_moveit_ready()) {
    response->success = false;
    response->message = "MoveIt not ready yet";
    return;
  }

  try {
    if (request->object_names.empty()) {
      // Remove all objects
      auto objects = planning_scene_interface_->getObjects();
      std::vector<std::string> all_object_names;
      for (const auto &[name, obj] : objects) {
        all_object_names.push_back(name);
      }
      planning_scene_interface_->removeCollisionObjects(all_object_names);
      response->success = true;
      response->message = "Removed all " +
                          std::to_string(all_object_names.size()) +
                          " collision objects";
    } else {
      // Remove specific objects
      planning_scene_interface_->removeCollisionObjects(request->object_names);
      response->success = true;
      response->message = "Removed " +
                          std::to_string(request->object_names.size()) +
                          " collision objects";
    }

  } catch (const std::exception &e) {
    response->success = false;
    response->message =
        "Failed to remove collision objects: " + std::string(e.what());
    RCLCPP_ERROR(this->get_logger(), "Remove collision object error: %s",
                 e.what());
  }
}

} // namespace moveit_wrapper
