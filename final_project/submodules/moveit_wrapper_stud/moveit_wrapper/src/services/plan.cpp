#include "moveit_wrapper/moveit_wrapper.hpp"

#include <future>
#include <mutex>

namespace moveit_wrapper {

void MoveitWrapper::plan_callback(
    const std::shared_ptr<moveit_wrapper_interfaces::srv::Plan::Request>
        request,
    std::shared_ptr<moveit_wrapper_interfaces::srv::Plan::Response> response) {

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

  if (request->waypoints.empty()) {
    response->success = false;
    response->message = "At least one waypoint is required";
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
      start_state.enforceBounds();
      planning_move_group->setStartState(start_state);
    } else {
      planning_move_group->setStartStateToCurrentState();
    }

    std::vector<moveit_msgs::msg::RobotTrajectory> trajectories;
    trajectories.reserve(request->waypoints.size());

    for (size_t i = 0; i < request->waypoints.size(); ++i) {
      const auto &waypoint = request->waypoints[i];

      apply_path_constraints(*planning_move_group, request->path_constraints,
                             waypoint.path_constraints);

      moveit::planning_interface::MoveGroupInterface::Plan plan;
      bool plan_success = false;

      if (!waypoint.named_target.empty()) {
        const auto &joint_values =
            planning_move_group->getNamedTargetValues(waypoint.named_target);
        planning_move_group->setJointValueTarget(joint_values);
        plan_success = (planning_move_group->plan(plan) ==
                        moveit::core::MoveItErrorCode::SUCCESS);

      } else {
        geometry_msgs::msg::PoseStamped target_pose = waypoint.pose;
        if (target_pose.header.frame_id !=
            planning_move_group->getPlanningFrame()) {
          target_pose = transform_pose(target_pose,
                                       planning_move_group->getPlanningFrame());
        }

        if (waypoint.linear_movement) {
          std::vector<geometry_msgs::msg::Pose> waypoints_vector{
              target_pose.pose};
          moveit_msgs::msg::RobotTrajectory trajectory;
          double fraction = planning_move_group->computeCartesianPath(
              waypoints_vector,
              (waypoint.linear_eef_step > 0 ? waypoint.linear_eef_step : 0.01),
              5.0, trajectory, !waypoint.linear_disable_collisions, nullptr);
          if (fraction > 0.95) {
            plan.trajectory_ = trajectory;
            plan_success = true;
          }
        } else {
          planning_move_group->setPoseTarget(target_pose.pose, ee_frame_);
          plan_success = (planning_move_group->plan(plan) ==
                          moveit::core::MoveItErrorCode::SUCCESS);
        }
      }

      if (!plan_success) {
        response->plans = trajectories;
        response->success = false;
        response->message =
            (request->send_partial ? "Partial planning - failed at waypoint "
                                   : "Planning failed at waypoint ") +
            std::to_string(i);
        return;
      }

      trajectories.push_back(plan.trajectory_);

      {
        std::lock_guard<std::mutex> plans_lock(plans_mutex_);
        stored_plans_.push_back(plan);
      }

      if (i < request->waypoints.size() - 1) {
        const auto &traj = plan.trajectory_.joint_trajectory;
        if (!traj.points.empty()) {
          moveit::core::RobotState end_state(
              planning_move_group->getRobotModel());

          end_state.setToDefaultValues();
          end_state.setVariablePositions(traj.joint_names,
                                         traj.points.back().positions);
          end_state.enforceBounds();

          planning_move_group->setStartState(end_state);

          RCLCPP_DEBUG(
              this->get_logger(),
              "Updated start state for next waypoint from end of "
              "trajectory %zu (joints: %zu, last point positions: %zu)",
              i, traj.joint_names.size(), traj.points.back().positions.size());
        } else {
          planning_move_group->setStartStateToCurrentState();
          RCLCPP_WARN(
              this->get_logger(),
              "Trajectory for waypoint %zu has no points; using current "
              "state as start for next waypoint",
              i);
        }
      }
    }

    response->plans = trajectories;
    response->success = true;
    response->message = "All waypoints planned successfully using group '" +
                        planning_move_group->getName() + "'";

  } catch (const std::exception &e) {
    response->success = false;
    response->message = "Planning error: " + std::string(e.what());
    RCLCPP_ERROR(this->get_logger(), "Planning error: %s", e.what());
  }
}

} // namespace moveit_wrapper
