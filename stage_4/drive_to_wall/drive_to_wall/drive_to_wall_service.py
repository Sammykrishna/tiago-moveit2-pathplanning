import time

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from custom_interfaces.srv import DriveToWallS


class DriveToWallServiceNode(Node):
    def __init__(self):
        super().__init__('drive_to_wall_service')

        # Parameters (you can adapt topic names if needed)
        self.declare_parameter('laser_topic', '/scan_raw')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')

        laser_topic = self.get_parameter('laser_topic').get_parameter_value().string_value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value

        # Laser sub + cmd_vel pub
        self.scan_sub = self.create_subscription(
            LaserScan,
            laser_topic,
            self.scan_callback,
            10
        )
        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)

        # For distance tracking
        self.min_range_ahead = None

        # Service: request/response = DriveToWallS
        self.srv = self.create_service(
            DriveToWallS,
            'drive_to_wall_service',
            self.handle_drive_to_wall
        )

        self.get_logger().info(
            f"DriveToWallServiceNode started. Laser: {laser_topic}, cmd_vel: {cmd_vel_topic}, "
            f"service: drive_to_wall_service"
        )

    def scan_callback(self, msg: LaserScan):
        """Update the minimum valid range in front of the robot."""
        valid_ranges = [r for r in msg.ranges if msg.range_min < r < msg.range_max]
        if not valid_ranges:
            self.min_range_ahead = None
        else:
            self.min_range_ahead = min(valid_ranges)

    def handle_drive_to_wall(self, request: DriveToWallS.Request, response: DriveToWallS.Response):
        """
        Service callback:
        - Use request.linear_x as speed
        - Use request.min_distance as stop distance
        - Drive until we reach that distance
        - Return final values in response
        """
        speed = float(request.linear_x)
        stop_distance = float(request.min_distance)

        self.get_logger().info(
            f"Service call received: speed={speed:.3f} m/s, stop at {stop_distance:.3f} m"
        )

        twist = Twist()

        # Simple control loop inside the service callback.
        # MultiThreadedExecutor lets scan_callback run in parallel.
        while rclpy.ok():
            if self.min_range_ahead is None:
                # No valid laser data: be safe and stop
                twist.linear.x = 0.0
                self.get_logger().warn("No valid laser data yet, keeping robot stopped.")
            elif self.min_range_ahead > stop_distance:
                # Still far from wall -> go forward
                twist.linear.x = speed
            else:
                # We reached (or are closer than) stop_distance -> stop and respond
                twist.linear.x = 0.0
                self.cmd_pub.publish(twist)

                self.get_logger().info(
                    f"Reached wall: distance={self.min_range_ahead:.3f} m "
                    f"(threshold {stop_distance:.3f} m). Stopping."
                )

                # Fill response
                response.linear_x = 0.0
                response.min_distance = float(self.min_range_ahead or 0.0)

                return response

            self.cmd_pub.publish(twist)
            time.sleep(0.1)  # 10 Hz loop

        
        twist.linear.x = 0.0
        self.cmd_pub.publish(twist)
        response.linear_x = 0.0
        response.min_distance = float(self.min_range_ahead or 0.0)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = DriveToWallServiceNode()


    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
