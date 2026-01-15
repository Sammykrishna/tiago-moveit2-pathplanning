import time

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from teleop_tools_msgs.action import Increment
from apriltag_msgs.msg import AprilTagDetectionArray
from custom_interfaces.action import LookAtMarker

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def det_to_int_id(det) -> int:
    _id = getattr(det, "id", None)
    if _id is None:
        return -1
    if isinstance(_id, int):
        return int(_id)
    try:
        return int(_id[0])
    except Exception:
        return -1


class LookAtMarkerActionServer(Node):
    def __init__(self):
        super().__init__("look_at_marker_action")

        self.declare_parameter("camera_frame", "head_front_camera_color_optical_frame")
        self.declare_parameter("tag_frame_prefix", "tag36h11_")
        self.declare_parameter("rate_hz", 15.0)
        self.declare_parameter("feedback_hz", 2.0)
        self.declare_parameter("deadband", 0.02)
        self.declare_parameter("gain_yaw", 0.8)
        self.declare_parameter("gain_pitch", 0.8)
        self.declare_parameter("max_step", 0.05)
        self.declare_parameter("overall_timeout_sec", 40.0)
        self.declare_parameter("acquire_timeout_sec", 8.0)

        self.camera_frame = self.get_parameter("camera_frame").get_parameter_value().string_value
        self.tag_frame_prefix = self.get_parameter("tag_frame_prefix").get_parameter_value().string_value
        self.rate_hz = float(self.get_parameter("rate_hz").value)
        self.feedback_hz = float(self.get_parameter("feedback_hz").value)
        self.deadband = float(self.get_parameter("deadband").value)
        self.gain_yaw = float(self.get_parameter("gain_yaw").value)
        self.gain_pitch = float(self.get_parameter("gain_pitch").value)
        self.max_step = float(self.get_parameter("max_step").value)
        self.overall_timeout_sec = float(self.get_parameter("overall_timeout_sec").value)
        self.acquire_timeout_sec = float(self.get_parameter("acquire_timeout_sec").value)

        self.visible_ids = set()
        self.cb_group = ReentrantCallbackGroup()

        self.tag_sub = self.create_subscription(
            AprilTagDetectionArray,
            "/tag_detections",
            self.detections_cb,
            10,
            callback_group=self.cb_group
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        self.increment_client = ActionClient(
            self,
            Increment,
            "/head_controller/increment",
            callback_group=self.cb_group
        )

        self.action_server = ActionServer(
            self,
            LookAtMarker,
            "/look_at_marker_action",
            execute_callback=self.execute_cb,
            goal_callback=self.goal_cb,
            cancel_callback=self.cancel_cb,
            callback_group=self.cb_group
        )

        self.get_logger().info("look_at_marker_action server ready on /look_at_marker_action")

    def detections_cb(self, msg: AprilTagDetectionArray):
        ids = set()
        for det in msg.detections:
            ids.add(det_to_int_id(det))
        self.visible_ids = ids

    def _sleep_dt(self, dt: float) -> None:
        try:
            self.get_clock().sleep_for(Duration(seconds=float(dt)))
        except Exception:
            time.sleep(float(dt))

    def goal_cb(self, goal_request):
        self.get_logger().info(f"Goal request: marker_id={goal_request.marker_id}")
        return GoalResponse.ACCEPT

    def cancel_cb(self, goal_handle):
        self.get_logger().warn("Cancel request received.")
        return CancelResponse.ACCEPT

    async def send_increment(self, yaw_inc: float, pitch_inc: float) -> bool:
        if not self.increment_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Increment action server '/head_controller/increment' not available")
            return False

        goal = Increment.Goal()
        goal.increment_by = [float(yaw_inc), float(pitch_inc)]

        goal_handle = await self.increment_client.send_goal_async(goal)
        if not goal_handle.accepted:
            self.get_logger().warn("Increment goal was rejected")
            return False

        await goal_handle.get_result_async()
        return True

    def _candidate_tag_frames(self, marker_id: int):
        base = f"{self.tag_frame_prefix}{marker_id}"
        return [
            base,
            f"{self.tag_frame_prefix}:{marker_id}",
            f"{self.tag_frame_prefix}{marker_id:02d}",
            f"tag_{marker_id}",
        ]

    async def execute_cb(self, goal_handle):
        marker_id = int(goal_handle.request.marker_id)
        feedback = LookAtMarker.Feedback()
        result = LookAtMarker.Result()

        dt = 1.0 / max(self.rate_hz, 1.0)
        fb_period = 1.0 / max(self.feedback_hz, 0.2)

        start_t = self.get_clock().now()
        last_fb_t = start_t
        no_tf_since = None

        candidates = self._candidate_tag_frames(marker_id)
        active_tag_frame = None

        self.get_logger().info(
            f"Executing: look at marker {marker_id} (camera_frame: {self.camera_frame}, tag candidates: {candidates})"
        )

        while True:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.message = "Canceled"
                return result

            now = self.get_clock().now()

            if (now - start_t).nanoseconds / 1e9 > self.overall_timeout_sec:
                goal_handle.abort()
                result.success = False
                result.message = "Timeout"
                return result

            if active_tag_frame is None:
                for cand in candidates:
                    try:
                        if self.tf_buffer.can_transform(
                            self.camera_frame, cand, rclpy.time.Time(), timeout=Duration(seconds=0.15)
                        ):
                            active_tag_frame = cand
                            self.get_logger().info(f"Using tag TF frame: {active_tag_frame}")
                            break
                    except Exception:
                        pass

            if active_tag_frame is None:
                if no_tf_since is None:
                    no_tf_since = now

                if (now - last_fb_t).nanoseconds / 1e9 >= fb_period:
                    feedback.err_x = 999.0
                    feedback.err_y = 999.0
                    goal_handle.publish_feedback(feedback)
                    last_fb_t = now

                if (now - no_tf_since).nanoseconds / 1e9 > self.acquire_timeout_sec:
                    goal_handle.abort()
                    result.success = False
                    result.message = (
                        "No TF for tag frame. Check AprilTag TF publishing and frame names. "
                        f"Tried tag frames: {candidates} with camera_frame='{self.camera_frame}'. "
                        "Tip: run `ros2 run tf2_ros tf2_echo <camera_frame> <tag_frame>` "
                        "or `ros2 run tf2_tools view_frames` to see available frames."
                    )
                    return result

                self._sleep_dt(dt)
                continue

            try:
                t = self.tf_buffer.lookup_transform(
                    self.camera_frame,
                    active_tag_frame,
                    rclpy.time.Time()
                )
                no_tf_since = None
            except TransformException:
                active_tag_frame = None
                self._sleep_dt(dt)
                continue

            tx = float(t.transform.translation.x)
            ty = float(t.transform.translation.y)
            tz = float(t.transform.translation.z)

            if tz <= 0.05:
                if (now - last_fb_t).nanoseconds / 1e9 >= fb_period:
                    feedback.err_x = 999.0
                    feedback.err_y = 999.0
                    goal_handle.publish_feedback(feedback)
                    last_fb_t = now
                self._sleep_dt(dt)
                continue

            err_x = tx / tz
            err_y = ty / tz

            if (now - last_fb_t).nanoseconds / 1e9 >= fb_period:
                feedback.err_x = err_x
                feedback.err_y = err_y
                goal_handle.publish_feedback(feedback)
                last_fb_t = now

            if abs(err_x) < self.deadband and abs(err_y) < self.deadband:
                goal_handle.succeed()
                result.success = True
                result.message = "Marker centered"
                return result

            yaw_inc = clamp(-self.gain_yaw * err_x, -self.max_step, self.max_step)
            pitch_inc = clamp(-self.gain_pitch * err_y, -self.max_step, self.max_step)

            ok = await self.send_increment(yaw_inc, pitch_inc)
            if not ok:
                goal_handle.abort()
                result.success = False
                result.message = "Increment action rejected or unavailable"
                return result

            self._sleep_dt(dt)


def main():
    rclpy.init()
    node = LookAtMarkerActionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == "__main__":
    main()