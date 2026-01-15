import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from sensor_msgs.msg import JointState
from teleop_tools_msgs.action import Increment

from custom_interfaces.action import MoveHead


class MoveHeadActionServer(Node):
    def __init__(self):
        super().__init__("move_head_action")

        self.cb_group = ReentrantCallbackGroup()

        self.current = {"head_1_joint": None, "head_2_joint": None}
        self.create_subscription(
            JointState, "/joint_states", self.joint_cb, 10,
            callback_group=self.cb_group
        )

        self.inc_client = ActionClient(
            self, Increment, "/head_controller/increment",
            callback_group=self.cb_group
        )

        self.server = ActionServer(
            self,
            MoveHead,
            "/move_head_action",
            execute_callback=self.execute_cb,
            goal_callback=self.goal_cb,
            cancel_callback=self.cancel_cb,
            callback_group=self.cb_group,
        )

        self.get_logger().info("✅ move_head_action server started on /move_head_action")

    def joint_cb(self, msg: JointState):
        name_to_pos = dict(zip(msg.name, msg.position))
        if "head_1_joint" in name_to_pos:
            self.current["head_1_joint"] = float(name_to_pos["head_1_joint"])
        if "head_2_joint" in name_to_pos:
            self.current["head_2_joint"] = float(name_to_pos["head_2_joint"])

    def goal_cb(self, goal_request: MoveHead.Goal):
        self.get_logger().info(
            f"Received goal: head_1={goal_request.target_head_1:.3f}, head_2={goal_request.target_head_2:.3f}"
        )
        return GoalResponse.ACCEPT

    def cancel_cb(self, goal_handle):
        self.get_logger().warn("Cancel requested.")
        return CancelResponse.ACCEPT

    def wait_for_joints(self, timeout_s=3.0):
        start = time.time()
        while rclpy.ok():
            if self.current["head_1_joint"] is not None and self.current["head_2_joint"] is not None:
                return True
            if time.time() - start > timeout_s:
                return False
            time.sleep(0.05)

    def clamp(self, v, lo, hi):
        return max(lo, min(hi, v))

    def send_increment(self, inc1: float, inc2: float) -> bool:
        if not self.inc_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Increment action server not available: /head_controller/increment")
            return False

        goal = Increment.Goal()
        goal.increment_by = [float(inc1), float(inc2)]
        future = self.inc_client.send_goal_async(goal)

        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("Increment goal rejected.")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return True

    def execute_cb(self, goal_handle):
        result = MoveHead.Result()

        if not self.wait_for_joints(timeout_s=3.0):
            result.success = False
            result.message = "No joint_states (head joints not found)."
            return result

        target1 = goal_handle.request.target_head_1
        target2 = goal_handle.request.target_head_2

        tol = 0.02
        step_max = 0.10
        deadline = time.time() + 15.0

        self.get_logger().info("Executing goal (absolute -> incremental steps)...")

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.message = "Goal canceled."
                return result

            c1 = self.current["head_1_joint"]
            c2 = self.current["head_2_joint"]

            fb = MoveHead.Feedback()
            fb.current_head_1 = float(c1)
            fb.current_head_2 = float(c2)
            goal_handle.publish_feedback(fb)

            e1 = target1 - c1
            e2 = target2 - c2

            if abs(e1) < tol and abs(e2) < tol:
                goal_handle.succeed()
                result.success = True
                result.message = "Reached target head angles."
                return result

            if time.time() > deadline:
                goal_handle.abort()
                result.success = False
                result.message = "Timeout while moving head."
                return result

            inc1 = self.clamp(e1, -step_max, step_max)
            inc2 = self.clamp(e2, -step_max, step_max)

            ok = self.send_increment(inc1, inc2)
            if not ok:
                goal_handle.abort()
                result.success = False
                result.message = "Failed to send increment command."
                return result

            time.sleep(0.05)


def main():
    rclpy.init()
    node = MoveHeadActionServer()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()