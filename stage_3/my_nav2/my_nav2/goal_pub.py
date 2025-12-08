import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class GoalPublisher(Node):
    def __init__(self):
        super().__init__('goal_pub')

        # Publisher to Nav2 goal topic
        self.pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

        # Parameters: pick any goal in map frame
        self.declare_parameter('x', 1.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('yaw', 0.0)  # radians

        # Use a timer to publish once after startup
        self.sent = False
        self.timer = self.create_timer(2.0, self.publish_once)

    def publish_once(self):
        if self.sent:
            return

        x = self.get_parameter('x').get_parameter_value().double_value
        y = self.get_parameter('y').get_parameter_value().double_value
        yaw = self.get_parameter('yaw').get_parameter_value().double_value

        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = 0.0

        # yaw -> quaternion (z,w)
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)

        self.pub.publish(msg)
        self.get_logger().info(
            f'Published Nav2 goal: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f} rad'
        )
        self.sent = True


def main(args=None):
    rclpy.init(args=args)
    node = GoalPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
