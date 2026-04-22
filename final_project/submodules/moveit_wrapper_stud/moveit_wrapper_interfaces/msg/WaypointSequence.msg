string sequence_id                                     # Identifier for this sequence
Waypoint[] waypoints                                   # Waypoints for this trajectory
moveit_msgs/Constraints path_constraints              # Constraints specific to this sequence
bool use_linear_interpolation                         # Use linear interpolation between waypoints
float64 velocity_scaling                               # Velocity scaling for this sequence (0.0 = use default)
float64 acceleration_scaling                           # Acceleration scaling for this sequence (0.0 = use default)
