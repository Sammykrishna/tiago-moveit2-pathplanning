import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class WallStopNode(Node):
    def __init__(self):
        super().__init__('wall_stop_node')

        
        self.declare_parameter('laser_topic', '/scan_raw')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('stop_distance', 0.6)   
        self.declare_parameter('forward_speed', 0.2)   

        laser_topic = self.get_parameter('laser_topic').get_parameter_value().string_value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value

        self.stop_distance = self.get_parameter('stop_distance').get_parameter_value().double_value
        self.forward_speed = self.get_parameter('forward_speed').get_parameter_value().double_value

        self.get_logger().info(
            f'WallStopNode started. Subscribing to {laser_topic}, publishing to {cmd_vel_topic}'
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            laser_topic,
            self.scan_callback,
            10
        )

        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)

        self.min_range_ahead = None
        self.timer = self.create_timer(0.1, self.control_loop)  # 10 Hz

    def scan_callback(self, msg: LaserScan):
        """
        Update the minimum distance in front of the robot.
        We look at all ranges (or you could crop to a frontal sector).
        """
        valid_ranges = [r for r in msg.ranges if msg.range_min < r < msg.range_max]
        if not valid_ranges:
            self.min_range_ahead = None
            return

        self.min_range_ahead = min(valid_ranges)

    def control_loop(self):
        """
        Simple control: drive forward until we are closer than stop_distance.
        """
        twist = Twist()

        if self.min_range_ahead is None:
            # No valid reading: be safe and stop
            self.get_logger().warn('No valid laser data, stopping.')
            twist.linear.x = 0.0
        elif self.min_range_ahead > self.stop_distance:
            # Far from wall -> drive forward
            twist.linear.x = self.forward_speed
        else:
            # Close to wall -> stop
            twist.linear.x = 0.0
            self.get_logger().info(
                f'Stopping, wall at {self.min_range_ahead:.2f} m (threshold {self.stop_distance:.2f} m)'
            )

        self.cmd_pub.publish(twist)

    @staticmethod
    def main(args=None):
        rclpy.init(args=args)
        node = WallStopNode()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()


def main(args=None):
    WallStopNode.main(args)
