import math
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Quaternion
from sensor_msgs.msg import LaserScan


def yaw_to_quaternion(z_yaw):
    # Convert yaw angle (radians) to quaternion representation (z-axis rotation only)
    half_yaw = z_yaw * 0.5
    return Quaternion(z=math.sin(half_yaw), w=math.cos(half_yaw))


class StartInspectionNode(Node):
    def __init__(self):
        super().__init__('start_inspection')

        self.set_parameters([
            Parameter('use_sim_time', Parameter.Type.BOOL, True)
        ])

        self.declare_parameter('start_x', -3.481)
        self.declare_parameter('start_y', 3.522)
        self.declare_parameter('start_yaw', 0.039)
        self.declare_parameter('goal_x', -0.525)
        self.declare_parameter('goal_y', 5.296)
        self.declare_parameter('goal_yaw', -0.19)
        self.declare_parameter('laser_topic', '/scan_raw')
        self.declare_parameter('front_open_distance', 2.0)

        self.start_x = self.get_parameter('start_x').value
        self.start_y = self.get_parameter('start_y').value
        self.start_yaw = self.get_parameter('start_yaw').value
        self.goal_x = self.get_parameter('goal_x').value
        self.goal_y = self.get_parameter('goal_y').value
        self.goal_yaw = self.get_parameter('goal_yaw').value
        self.laser_topic = self.get_parameter('laser_topic').value
        self.front_open_distance = self.get_parameter('front_open_distance').value

        self.initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10
        )
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

        self.scan_sub = self.create_subscription(
            LaserScan,
            self.laser_topic,
            self.scan_callback,
            10
        )

        self.amcl_ready = False
        self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self._amcl_callback,
            10
        )

        self.initial_pose_repeat = 0
        self.goal_sent = False
        self.door_open = False

        self.create_timer(1.0, self.publish_initial_pose_and_goal)

        self.get_logger().info("start_inspection node started")

    def _amcl_callback(self, msg):
        # Detect AMCL readiness by monitoring its pose publications
        if not self.amcl_ready:
            self.amcl_ready = True

    def scan_callback(self, msg: LaserScan):
        # Analyze front-facing laser ranges to detect door opening
        if self.door_open:
            return

        n = len(msg.ranges)
        if n == 0:
            return

        center = n // 2
        window_indices = range(max(0, center - 5), min(n, center + 6))
        valid = [
            msg.ranges[i]
            for i in window_indices
            if msg.range_min < msg.ranges[i] < msg.range_max
        ]

        if not valid:
            return

        min_front = min(valid)
        if min_front > self.front_open_distance:
            self.door_open = True

    def publish_initial_pose_and_goal(self):
        # Publish initial pose repeatedly, then dispatch goal after AMCL readiness and door detection
        if self.get_clock().now().nanoseconds == 0:
            return

        if self.initial_pose_repeat < 5:
            msg = PoseWithCovarianceStamped()
            msg.header.frame_id = 'map'
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.pose.pose.position.x = self.start_x
            msg.pose.pose.position.y = self.start_y
            msg.pose.pose.orientation = yaw_to_quaternion(self.start_yaw)
            msg.pose.covariance[0] = 0.05
            msg.pose.covariance[7] = 0.05
            msg.pose.covariance[35] = 0.1

            self.initialpose_pub.publish(msg)
            self.initial_pose_repeat += 1
            return

        if not self.amcl_ready:
            return

        if not self.door_open:
            return

        if not self.goal_sent:
            goal = PoseStamped()
            goal.header.frame_id = 'map'
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.pose.position.x = self.goal_x
            goal.pose.position.y = self.goal_y
            goal.pose.orientation = yaw_to_quaternion(self.goal_yaw)

            self.goal_pub.publish(goal)
            self.goal_sent = True
            self.get_logger().info("Navigation goal sent successfully")


def main(args=None):
    rclpy.init(args=args)
    node = StartInspectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()