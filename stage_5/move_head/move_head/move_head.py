import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from sensor_msgs.msg import JointState
from teleop_tools_msgs.action import Increment


class MoveHead(Node):
    def __init__(self):
        super().__init__("move_head")

        self.client = ActionClient(self, Increment, "/head_controller/increment")

        self.current = {"head_1_joint": None, "head_2_joint": None}
        self.create_subscription(JointState, "/joint_states", self.joint_cb, 10)

    def joint_cb(self, msg: JointState):
        name_to_pos = dict(zip(msg.name, msg.position))
        if "head_1_joint" in name_to_pos:
            self.current["head_1_joint"] = float(name_to_pos["head_1_joint"])
        if "head_2_joint" in name_to_pos:
            self.current["head_2_joint"] = float(name_to_pos["head_2_joint"])

    def have_joints(self):
        return (self.current["head_1_joint"] is not None) and (self.current["head_2_joint"] is not None)

    def clamp(self, v, lo, hi):
        return max(lo, min(hi, v))

    def move_to_absolute(self, target_head1: float, target_head2: float,
                         step_max: float = 0.10, tol: float = 0.02, timeout_s: float = 10.0):

        start = self.get_clock().now()
        while rclpy.ok() and not self.have_joints():
            if (self.get_clock().now() - start).nanoseconds / 1e9 > 3.0:
                self.get_logger().error("No /joint_states received (head joints not found).")
                return False
            rclpy.spin_once(self, timeout_sec=0.1)

        if not self.client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error("Action server /head_controller/increment not available.")
            return False

        start_move = self.get_clock().now()
        while rclpy.ok():
            c1 = self.current["head_1_joint"]
            c2 = self.current["head_2_joint"]

            e1 = target_head1 - c1
            e2 = target_head2 - c2

            if abs(e1) < tol and abs(e2) < tol:
                self.get_logger().info(f"Reached target head angles: head_1={c1:.3f}, head_2={c2:.3f}")
                return True

            inc1 = self.clamp(e1, -step_max, step_max)
            inc2 = self.clamp(e2, -step_max, step_max)

            goal = Increment.Goal()
            goal.increment_by = [float(inc1), float(inc2)]

            send_future = self.client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, send_future)

            goal_handle = send_future.result()
            if not goal_handle or not goal_handle.accepted:
                self.get_logger().error("Goal rejected by /head_controller/increment.")
                return False

            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)

            if (self.get_clock().now() - start_move).nanoseconds / 1e9 > timeout_s:
                self.get_logger().error("Timeout while moving head.")
                return False

            rclpy.spin_once(self, timeout_sec=0.05)


def main():
    rclpy.init()
    node = MoveHead()

    ok = node.move_to_absolute(target_head1=0.3, target_head2=-0.5)

    node.get_logger().info(f"Move finished: success={ok}")
    node.destroy_node()
    rclpy.shutdown()