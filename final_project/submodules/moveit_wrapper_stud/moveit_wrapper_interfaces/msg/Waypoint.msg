geometry_msgs/PoseStamped pose         #pose of waypoint, used when named_target is empty
string named_target                    #robot states available as targets, both either remembered states or default states from srdf.

# Linear movement
bool linear_movement            #true to move the arm on a straigt line to the pose
float32 linear_eef_step         #step size in [m] to generate linear movement (default 0.01)
float32 linear_jump_thresh      #The jump_threshold specifies the maximum distance in configuration space between consecutive points in the resulting path.
bool linear_disable_collisions  #Just don't set it to true if you don't know what you are doing :)

moveit_msgs/Constraints path_constraints    #list of constraints for this movement
