import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

from custom_interfaces.msg import DriveToWallT, ReachedWallT


class DriveToWallTopic(Node):
    def __init__(self):
        super().__init__('drive_to_wall_topic')

        # Parameters (you can adapt to your robot)
        self.declare_parameter('laser_topic', '/scan_raw')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')

        laser_topic = self.get_parameter('laser_topic').get_parameter_value().string_value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value

        # Internal state
        self.active = False           # drive command received?
        self.target_speed = 0.0
        self.stop_distance = 0.5
        self.min_range_ahead = None

        # Subscriptions / publishers
        self.drive_cmd_sub = self.create_subscription(
            DriveToWallT,
            'drive_to_wall_cmd',      # topic for custom command
            self.drive_cmd_callback,
            10
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            laser_topic,
            self.scan_callback,
            10
        )

        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.result_pub = self.create_publisher(ReachedWallT, 'reached_wall', 10)

        # Control loop timer (10 Hz)
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info(
            f'drive_to_wall_topic started. Listening on drive_to_wall_cmd, '
            f'laser: {laser_topic}, cmd_vel: {cmd_vel_topic}'
        )

    # --- Callbacks ---

    def drive_cmd_callback(self, msg: DriveToWallT):
        self.target_speed = msg.linear_x
        self.stop_distance = msg.min_distance
        self.active = True
        self.get_logger().info(
            f'Received DriveToWallT: speed={msg.linear_x:.2f} m/s, '
            f'stop_distance={msg.min_distance:.2f} m'
        )

    def scan_callback(self, msg: LaserScan):
        valid = [r for r in msg.ranges if msg.range_min < r < msg.range_max]
        self.min_range_ahead = min(valid) if valid else None

    def control_loop(self):
        if not self.active:
            return  # waiting for command

        twist = Twist()

        if self.min_range_ahead is None:
            # No data yet, be safe
            self.get_logger().warn('No laser data yet, stopping.')
            twist.linear.x = 0.0
            self.cmd_pub.publish(twist)
            return

        if self.min_range_ahead > self.stop_distance:
            # Still far → drive forward
            twist.linear.x = self.target_speed
            self.cmd_pub.publish(twist)
        else:
            # Close enough → stop and report success
            self.get_logger().info(
                f'Reached wall at {self.min_range_ahead:.2f} m '
                f'(threshold {self.stop_distance:.2f} m), stopping.'
            )
            twist.linear.x = 0.0
            self.cmd_pub.publish(twist)

            result = ReachedWallT()
            result.message = 'reached wall'
            result.success = True
            self.result_pub.publish(result)

            # Only do this once per command
            self.active = False


def main(args=None):
    rclpy.init(args=args)
    node = DriveToWallTopic()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
