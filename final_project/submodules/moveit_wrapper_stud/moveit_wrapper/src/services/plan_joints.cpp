#include "moveit_wrapper/moveit_wrapper.hpp"

#include <future>
#include <map>
#include <mutex>

namespace moveit_wrapper {

void MoveitWrapper::plan_joints_callback(
    const std::shared_ptr<moveit_wrapper_interfaces::srv::PlanJoints::Request>
        request,
    std::shared_ptr<moveit_wrapper_interfaces::srv::PlanJoints::Response>
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

  auto planning_move_group = get_move_group(request->move_group);
  if (!planning_move_group) {
    response->success = false;
    response->message =
        "Planning group '" + request->move_group + "' not found. Available: ";
    for (const auto &[name, group] : move_groups_) {
      (void)group;
      response->message += name + " ";
    }
    return;
  }

  try {
    if (request->use_start_state) {
      moveit::core::RobotState start_state(
          planning_move_group->getRobotModel());
      moveit::core::robotStateMsgToRobotState(request->start_state,
                                              start_state);
      planning_move_group->setStartState(start_state);
    }

    apply_path_constraints(*planning_move_group, request->path_constraints,
                           moveit_msgs::msg::Constraints());

    std::vector<moveit_msgs::msg::RobotTrajectory> trajectories;

    for (size_t i = 0; i < request->joint_states.size(); ++i) {
      const auto &joint_state = request->joint_states[i];

      std::map<std::string, double> joint_values;
      for (size_t j = 0;
           j < joint_state.name.size() && j < joint_state.position.size();
           ++j) {
        joint_values[joint_state.name[j]] = joint_state.position[j];
      }

      planning_move_group->setJointValueTarget(joint_values);

      moveit::planning_interface::MoveGroupInterface::Plan plan;
      bool plan_success = (planning_move_group->plan(plan) ==
                           moveit::core::MoveItErrorCode::SUCCESS);

      if (plan_success) {
        trajectories.push_back(plan.trajectory_);

        {
          std::lock_guard<std::mutex> plans_lock(plans_mutex_);
          stored_plans_.push_back(plan);
        }

        if (i < request->joint_states.size() - 1) {
          planning_move_group->setStartStateToCurrentState();
        }
      } else {
        if (request->send_partial) {
          response->plans = trajectories;
          response->success = false;
          response->message = "Partial planning - failed at joint waypoint " +
                              std::to_string(i) + " using group '" +
                              planning_move_group->getName() + "'";
          return;
        } else {
          response->success = false;
          response->message = "Planning failed at joint waypoint " +
                              std::to_string(i) + " using group '" +
                              planning_move_group->getName() + "'";
          return;
        }
      }
    }

    response->success = true;
    response->message =
        "All joint waypoints planned successfully using group '" +
        planning_move_group->getName() + "'";
    response->plans = trajectories;

  } catch (const std::exception &e) {
    response->success = false;
    response->message = "Planning error: " + std::string(e.what());
    RCLCPP_ERROR(this->get_logger(), "Planning error: %s", e.what());
  }
}

} // namespace moveit_wrapper
