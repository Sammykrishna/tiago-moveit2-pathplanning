import rclpy
from rclpy.node import Node

from custom_interfaces.msg import DriveToWallT, ReachedWallT


class DriveToWallTopicClient(Node):
    def __init__(self):
        super().__init__('drive_to_wall_topic_client')

        self.cmd_pub = self.create_publisher(DriveToWallT, 'drive_to_wall_cmd', 10)
        self.result_sub = self.create_subscription(
            ReachedWallT,
            'reached_wall',
            self.result_callback,
            10
        )

        self.sent = False
        self.timer = self.create_timer(1.0, self.timer_callback)

        self.get_logger().info('drive_to_wall_topic_client started.')

    def timer_callback(self):
        if self.sent:
            return

        msg = DriveToWallT()
        msg.linear_x = 0.25      # forward speed (m/s)
        msg.min_distance = 0.7   # stop 70 cm before wall

        self.cmd_pub.publish(msg)
        self.sent = True
        self.get_logger().info(
            f'Published DriveToWallT: speed={msg.linear_x:.2f}, '
            f'stop_distance={msg.min_distance:.2f}'
        )

    def result_callback(self, msg: ReachedWallT):
        self.get_logger().info(
            f'Received reached_wall: success={msg.success}, '
            f'message="{msg.message}"'
        )
        if msg.success:
            self.get_logger().info('Inspection done, shutting down client.')
            # clean shutdown
            self.destroy_node()
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = DriveToWallTopicClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    # rclpy.shutdown() is called in result_callback when finished
