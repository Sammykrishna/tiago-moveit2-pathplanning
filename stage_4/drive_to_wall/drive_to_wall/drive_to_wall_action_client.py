import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from custom_interfaces.action import DriveToWallA


class DriveToWallActionClient(Node):
    def __init__(self):
        super().__init__('drive_to_wall_action_client')

        self._action_client = ActionClient(self, DriveToWallA, 'drive_to_wall_action')
        self._goal_handle = None
        self._cancel_sent = False

        self.get_logger().info("Waiting for action server 'drive_to_wall_action'...")
        self._action_client.wait_for_server()
        self.get_logger().info("Action server available, sending goal...")

        # Define goal (adapt if you want)
        goal_msg = DriveToWallA.Goal()
        goal_msg.linear_x = 0.2       # speed
        goal_msg.min_distance = 0.6   # desired stop distance

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            rclpy.shutdown()
            return

        self.get_logger().info('Goal accepted :)')
        self._goal_handle = goal_handle

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        current_distance = feedback.current_distance
        self.get_logger().info(
            f'Feedback: current distance = {current_distance:.3f} m'
        )

      
        if (not self._cancel_sent
                and current_distance != float('inf')
                and current_distance < 1.5):
            self._cancel_sent = True
            self.get_logger().info(
                f"Current distance {current_distance:.3f} < 1.5 m -> sending cancel request!"
            )
            cancel_future = self._goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.cancel_done_callback)

    def cancel_done_callback(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('Goal successfully canceled on server.')
        else:
            self.get_logger().warn('Goal cancel request rejected / no goals canceled.')

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(
            f"Result received: message='{result.message}', success={result.success}"
        )

        
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = DriveToWallActionClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
