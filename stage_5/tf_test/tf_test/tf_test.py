import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import Twist
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class TfApproachTag(Node):
    def __init__(self):
        super().__init__("tf_test")

        self.source_frame = "tag36h11_3"

        self.target_frame = "base_link"

        self.desired_x = 1.0
        self.xy_tolerance = 0.05

        self.k_lin = 0.6
        self.k_ang = 1.8
        self.max_v = 0.25
        self.max_w = 1.2

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(0.1, self.on_timer)

        self.get_logger().info(
            f"Approaching {self.source_frame} using target frame {self.target_frame}. "
            f"Stopping at x={self.desired_x:.2f}m, y≈0."
        )

    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)

    def on_timer(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.source_frame,
                Time()
            )
        except TransformException as ex:
            self.get_logger().warn(
                f"TF missing {self.source_frame} -> {self.target_frame}: {ex}"
            )
            self.stop_robot()
            return

        x = tf.transform.translation.x
        y = tf.transform.translation.y

        yaw_err = math.atan2(y, x)

        x_err = x - self.desired_x

        if abs(x_err) < self.xy_tolerance and abs(y) < self.xy_tolerance:
            self.get_logger().info(f"Reached target: x={x:.2f}, y={y:.2f}. Stopping.")
            self.stop_robot()
            return

        w = clamp(self.k_ang * yaw_err, -self.max_w, self.max_w)

        if abs(yaw_err) > 0.35:
            v = 0.0
        else:
            v = clamp(self.k_lin * x_err, 0.0, self.max_v)

        twist = Twist()
        twist.linear.x = v
        twist.angular.z = w
        self.cmd_pub.publish(twist)

        self.get_logger().info(f"tag in {self.target_frame}: x={x:.2f}, y={y:.2f}, v={v:.2f}, w={w:.2f}")


def main(args=None):
    rclpy.init(args=args)
    node = TfApproachTag()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()