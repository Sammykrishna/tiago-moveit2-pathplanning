import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from sensor_msgs.msg import LaserScan


def yaw_to_quaternion(z_yaw):
    """Convert yaw (in radians) to a geometry_msgs/Quaternion."""
    
    half_yaw = z_yaw * 0.5
    qz = math.sin(half_yaw)
    qw = math.cos(half_yaw)
    from geometry_msgs.msg import Quaternion
    q = Quaternion()
    q.z = qz
    q.w = qw
    return q


class StartInspectionNode(Node):
    def __init__(self):
        super().__init__('start_inspection')


        self.declare_parameter('start_x', -1.451)
        self.declare_parameter('start_y', -5.25)
        self.declare_parameter('start_yaw', 1.57)  # rad

     
        self.declare_parameter('goal_x', -3.1730931817054753)     #
        self.declare_parameter('goal_y', 1.3715826037665333)     
        self.declare_parameter('goal_yaw', 0.7168599102945101)  

        
        self.declare_parameter('laser_topic', '/scan_raw')
        self.declare_parameter('front_open_distance', 2.0)  # m

        # --- Load parameters ---
        self.start_x = self.get_parameter('start_x').get_parameter_value().double_value
        self.start_y = self.get_parameter('start_y').get_parameter_value().double_value
        self.start_yaw = self.get_parameter('start_yaw').get_parameter_value().double_value

        self.goal_x = self.get_parameter('goal_x').get_parameter_value().double_value
        self.goal_y = self.get_parameter('goal_y').get_parameter_value().double_value
        self.goal_yaw = self.get_parameter('goal_yaw').get_parameter_value().double_value

        self.laser_topic = self.get_parameter('laser_topic').get_parameter_value().string_value
        self.front_open_distance = (
            self.get_parameter('front_open_distance').get_parameter_value().double_value
        )

        # --- Publishers & subscribers ---
        self.initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )

        self.goal_pub = self.create_publisher(
            PoseStamped,
            '/goal_pose',
            10
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            self.laser_topic,
            self.scan_callback,
            10
        )

        
        self.initial_pose_sent = False
        self.door_open = False
        self.goal_sent = False

        
        self.pose_timer = self.create_timer(1.0, self.publish_initial_pose)

        self.get_logger().info(
            f"start_inspection node started. Using laser topic '{self.laser_topic}'."
        )

    # ---------- initial pose ----------
    def publish_initial_pose(self):
        if self.initial_pose_sent:
            return

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = self.start_x
        msg.pose.pose.position.y = self.start_y
        msg.pose.pose.orientation = yaw_to_quaternion(self.start_yaw)

        # simple small covariance
        msg.pose.covariance[0] = 0.05  # x
        msg.pose.covariance[7] = 0.05  # y
        msg.pose.covariance[35] = 0.1  # yaw

        self.initialpose_pub.publish(msg)
        self.get_logger().info(
            f"Publishing initial pose at ({self.start_x:.2f}, {self.start_y:.2f}, yaw={self.start_yaw:.2f})"
        )


        if not hasattr(self, 'pose_publish_count'):
            self.pose_publish_count = 0
        self.pose_publish_count += 1
        if self.pose_publish_count >= 5:
            self.initial_pose_sent = True
            self.pose_timer.cancel()
            self.get_logger().info("Initial pose publishing done.")

    # ---------- door detection ----------
    def scan_callback(self, msg: LaserScan):
        # consider a small window around the front direction
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

        if not self.door_open:
            self.get_logger().debug(f"Front distance: {min_front:.2f} m")

            if min_front > self.front_open_distance:
                self.door_open = True
                self.get_logger().info(
                    f"Door seems open (front distance {min_front:.2f} m > {self.front_open_distance:.2f} m)."
                )
                self.send_goal()

    # ---------- send navigation goal ----------
    def send_goal(self):
        if self.goal_sent:
            return

        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = self.goal_x
        goal.pose.position.y = self.goal_y
        goal.pose.orientation = yaw_to_quaternion(self.goal_yaw)

        self.goal_pub.publish(goal)
        self.goal_sent = True
        self.get_logger().info(
            f"Published Nav2 goal to ({self.goal_x:.2f}, {self.goal_y:.2f}, yaw={self.goal_yaw:.2f})."
        )


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
