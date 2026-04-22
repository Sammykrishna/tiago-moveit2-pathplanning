# MoveIt Wrapper

**MoveIt Wrapper** is a modular, ROS 2 Humble component designed to simplify and extend MoveIt 2 for advanced motion planning, parallel planning, and collision scene management in manipulation tasks.

---

## Features

- **ROS 2 Component:**  
  Easily launch as a component or in a container. Compatible with composable nodes.

- **Service API for Planning:**  
  Plan Cartesian or joint space trajectories via ROS 2 services:

  - `plan` — Plan pose waypoints (supports linear or constrained segments).
  - `plan_joints` — Plan via joint space waypoints.
  - `execute_plans` — Execute a list of planned trajectories.

- **Collision Object Management:**

  - Add, remove, attach, and detach collision objects via services.
  - Query the current scene for collision/attached objects (as `visualization_msgs/Marker[]` for visualization).

- **Parallel Planning Action:**  
  Use the `plan_parallel` action to plan multiple trajectories or with multiple planners in parallel.  
  Supports planner selection, custom quality metrics, early stopping, and detailed result statistics.

- **YAML-Configurable Parameters:**  
  All key parameters (planning group, velocity/acceleration scaling, parallel planner configs, etc.) are loaded from YAML and can be reconfigured at runtime.
