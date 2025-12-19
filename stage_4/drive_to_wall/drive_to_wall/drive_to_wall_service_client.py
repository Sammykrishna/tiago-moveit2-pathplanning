import rclpy
from rclpy.node import Node

from custom_interfaces.srv import DriveToWallS


class DriveToWallServiceClient(Node):
    def __init__(self):
        super().__init__('drive_to_wall_service_client')

        self.cli = self.create_client(DriveToWallS, 'drive_to_wall_service')
        self.get_logger().info("Waiting for service 'drive_to_wall_service'...")

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting...")

        self.get_logger().info("Service available, sending request...")

        # Prepare request (adjust these numbers if needed)
        request = DriveToWallS.Request()
        request.linear_x = 0.2       # speed
        request.min_distance = 0.6   # stop distance

        self.future = self.cli.call_async(request)
        self.future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
        else:
            self.get_logger().info(
                f"Service response: final speed={response.linear_x:.3f}, "
                f"final distance={response.min_distance:.3f} m"
            )
        # Exit the node after receiving response
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = DriveToWallServiceClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    # rclpy.shutdown() is called in response_callback
