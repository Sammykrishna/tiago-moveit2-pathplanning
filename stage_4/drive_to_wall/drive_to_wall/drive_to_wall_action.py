import time

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionServer, CancelResponse, GoalResponse

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

from custom_interfaces.action import DriveToWallA


class DriveToWallActionNode(Node):
    def __init__(self):
        super().__init__('drive_to_wall_action')

        # Parameters so you can switch /scan_raw <-> /scan easily
        self.declare_parameter('laser_topic', '/scan_raw')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')

        laser_topic = self.get_parameter('laser_topic').get_parameter_value().string_value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value

        # Sub + Pub
        self.scan_sub = self.create_subscription(
            LaserScan,
            laser_topic,
            self.scan_callback,
            10
        )
        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)

        self.min_range_ahead = None

        # Action server
        self._action_server = ActionServer(
            self,
            DriveToWallA,
            'drive_to_wall_action',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        self.get_logger().info(
            f"DriveToWallActionNode started. Laser: {laser_topic}, "
            f"cmd_vel: {cmd_vel_topic}, action: drive_to_wall_action"
        )

    # ---------- Laser callback ----------

    def scan_callback(self, msg: LaserScan):
        valid_ranges = [r for r in msg.ranges if msg.range_min < r < msg.range_max]
        if not valid_ranges:
            self.min_range_ahead = None
        else:
            self.min_range_ahead = min(valid_ranges)

    # ---------- Action callbacks ----------

    def goal_callback(self, goal_request: DriveToWallA.Goal):
        self.get_logger().info(
            f"Received goal: speed={goal_request.linear_x:.3f}, "
            f"min_dist={goal_request.min_distance:.3f}"
        )
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received request to cancel goal')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
     
        goal = goal_handle.request
        speed = float(goal.linear_x)
        stop_distance = float(goal.min_distance)

        self.get_logger().info(
            f"Executing goal: speed={speed:.3f}, stop_dist={stop_distance:.3f}"
        )

        twist = Twist()
        feedback_msg = DriveToWallA.Feedback()

        while rclpy.ok():
            # Check cancel
            if goal_handle.is_cancel_requested:
                self.get_logger().info('Goal canceled by client. Stopping robot.')
                twist.linear.x = 0.0
                self.cmd_pub.publish(twist)

                goal_handle.canceled()
                result = DriveToWallA.Result()
                result.message = 'Goal canceled'
                result.success = False
                return result

            # Behavior based on laser
            if self.min_range_ahead is None:
                # No valid scan yet -> stop and send "infinite" distance
                twist.linear.x = 0.0
                self.cmd_pub.publish(twist)
                feedback_msg.current_distance = float('inf')
                self.get_logger().warn('No valid laser data yet.')
                goal_handle.publish_feedback(feedback_msg)

            elif self.min_range_ahead > stop_distance:
                # Still far from wall -> drive
                twist.linear.x = speed
                self.cmd_pub.publish(twist)

                feedback_msg.current_distance = self.min_range_ahead
                goal_handle.publish_feedback(feedback_msg)

            else:
                # We are close enough -> stop and finish
                twist.linear.x = 0.0
                self.cmd_pub.publish(twist)

                self.get_logger().info(
                    f"Reached wall at distance={self.min_range_ahead:.3f} m "
                    f"(threshold {stop_distance:.3f} m)."
                )

                result = DriveToWallA.Result()
                result.message = 'reached wall'
                result.success = True
                goal_handle.succeed()
                return result

            time.sleep(0.1)  # 10 Hz loop

        # If node is shutting down
        twist.linear.x = 0.0
        self.cmd_pub.publish(twist)
        result = DriveToWallA.Result()
        result.message = 'aborted (shutdown)'
        result.success = False
        return result


def main(args=None):
    rclpy.init(args=args)
    node = DriveToWallActionNode()

    # Executor with multiple threads so subscriptions + actions run nicely
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
