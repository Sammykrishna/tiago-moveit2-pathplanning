// moveit_wrapper/src/services/detach_object.cpp
#include "moveit_wrapper/moveit_wrapper.hpp"

#include <mutex>
#include <string>
#include <vector>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

namespace moveit_wrapper {

void MoveitWrapper::detach_object_callback(
    const std::shared_ptr<moveit_wrapper_interfaces::srv::DettachObj::Request>
        request,
    std::shared_ptr<moveit_wrapper_interfaces::srv::DettachObj::Response>
        response) {

  if (!is_moveit_ready()) {
    response->success = false;
    response->message = "MoveIt not ready yet";
    return;
  }

  if (request->move_group.empty()) {
    response->success = false;
    response->message = "move_group must be specified to detach objects";
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

    std::vector<std::string> to_detach = request->object_names;

    if (to_detach.empty()) {
      auto attached_objects = planning_scene_interface_->getAttachedObjects();
      for (const auto &[name, _] : attached_objects) {
        to_detach.push_back(name);
      }
    }

    if (to_detach.empty()) {
      response->success = false;
      response->message = "No attached objects to detach";
      return;
    }

    std::size_t detached_count = 0;
    std::size_t readded_count = 0;

    for (const std::string &object_name : to_detach) {
      moveit_msgs::msg::CollisionObject world_obj;
      bool have_attached_obj = false;

      {
        auto attached_map =
            planning_scene_interface_->getAttachedObjects({object_name});
        auto it = attached_map.find(object_name);
        if (it != attached_map.end()) {
          world_obj = it->second.object;
          world_obj.id = object_name;
          world_obj.operation = moveit_msgs::msg::CollisionObject::ADD;
          have_attached_obj = true;
        }
      }

      target_move_group->detachObject(object_name);
      detached_count++;

      moveit_msgs::msg::AttachedCollisionObject aco_remove;
      aco_remove.object.id = object_name;
      aco_remove.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
      planning_scene_interface_->applyAttachedCollisionObject(aco_remove);

      if (have_attached_obj) {
        planning_scene_interface_->applyCollisionObject(world_obj);
        readded_count++;
      } else {
        RCLCPP_WARN(this->get_logger(),
                    "detach_object_callback: object '%s' was not found in "
                    "attached objects; cannot re-add geometry to world",
                    object_name.c_str());
      }
    }

    response->success = true;
    response->message = "Detached " + std::to_string(detached_count) +
                        " objects, re-added " + std::to_string(readded_count) +
                        " to world";

  } catch (const std::exception &e) {
    response->success = false;
    response->message = "Failed to detach objects: " + std::string(e.what());
    RCLCPP_ERROR(this->get_logger(), "Detach object error: %s", e.what());
  }
}

} // namespace moveit_wrapper
