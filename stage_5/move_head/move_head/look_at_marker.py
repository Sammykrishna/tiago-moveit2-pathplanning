import math
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState

from rclpy.action import ActionClient
from teleop_tools_msgs.action import Increment

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from apriltag_msgs.msg import AprilTagDetectionArray


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class LookAtMarker(Node):
    def __init__(self):
        super().__init__('look_at_marker')

        self.declare_parameter('marker_id', 3)
        self.declare_parameter('camera_frame', 'head_front_camera_color_optical_frame')
        self.declare_parameter('tag_prefix', 'tag36h11_')
        self.declare_parameter('seen_timeout', 0.6)

        self.declare_parameter('gain_yaw', 1.2)
        self.declare_parameter('gain_pitch', 1.2)
        self.declare_parameter('max_step', 0.15)
        self.declare_parameter('tol_yaw', 0.03)
        self.declare_parameter('tol_pitch', 0.03)

        self.declare_parameter('yaw_sign', 1.0)
        self.declare_parameter('pitch_sign', 1.0)

        self.marker_id = int(self.get_parameter('marker_id').value)
        self.camera_frame = str(self.get_parameter('camera_frame').value)
        self.tag_prefix = str(self.get_parameter('tag_prefix').value)
        self.tag_frame = f"{self.tag_prefix}{self.marker_id}"

        self.seen_timeout = float(self.get_parameter('seen_timeout').value)

        self.gain_yaw = float(self.get_parameter('gain_yaw').value)
        self.gain_pitch = float(self.get_parameter('gain_pitch').value)
        self.max_step = float(self.get_parameter('max_step').value)
        self.tol_yaw = float(self.get_parameter('tol_yaw').value)
        self.tol_pitch = float(self.get_parameter('tol_pitch').value)
        self.yaw_sign = float(self.get_parameter('yaw_sign').value)
        self.pitch_sign = float(self.get_parameter('pitch_sign').value)

        self.get_logger().info(
            f"Looking at marker id={self.marker_id}. "
            f"Using /detections + /head_controller/increment, TF {self.camera_frame} -> {self.tag_frame}"
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.inc_client = ActionClient(self, Increment, '/head_controller/increment')

        self.last_seen = None
        self.create_subscription(AprilTagDetectionArray, '/detections', self.detections_cb, 10)
        self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)

        self.joint_map = {}
        self.timer = self.create_timer(0.15, self.control_loop)

    def joint_cb(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            self.joint_map[name] = pos

    def _det_id(self, det):
        v = getattr(det, 'id', None)
        if v is None:
            return None
        if isinstance(v, (list, tuple)):
            return int(v[0]) if len(v) > 0 else None
        return int(v)

    def detections_cb(self, msg: AprilTagDetectionArray):
        for det in msg.detections:
            if self._det_id(det) == self.marker_id:
                self.last_seen = self.get_clock().now()
                return

    def send_increment(self, head1_inc, head2_inc):
        goal = Increment.Goal()
        goal.increment_by = [float(head1_inc), float(head2_inc)]

        if not self.inc_client.wait_for_server(timeout_sec=0.2):
            self.get_logger().warn("Action server /head_controller/increment not available yet")
            return

        self.inc_client.send_goal_async(goal)

    def control_loop(self):
        if self.last_seen is None:
            return

        age = (self.get_clock().now() - self.last_seen).nanoseconds * 1e-9
        if age > self.seen_timeout:
            return

        try:
            t = self.tf_buffer.lookup_transform(
                self.camera_frame,
                self.tag_frame,
                rclpy.time.Time()
            )
        except TransformException as ex:
            self.get_logger().warn(f"TF not ready {self.camera_frame} <- {self.tag_frame}: {ex}")
            return

        x = t.transform.translation.x
        y = t.transform.translation.y
        z = t.transform.translation.z

        if z <= 0.05:
            return

        yaw_err = math.atan2(x, z)
        pitch_err = math.atan2(y, z)

        if abs(yaw_err) < self.tol_yaw and abs(pitch_err) < self.tol_pitch:
            self.get_logger().info("Marker centered ✅")
            return

        head1_inc = self.yaw_sign * (-self.gain_yaw * yaw_err)
        head2_inc = self.pitch_sign * (-self.gain_pitch * pitch_err)

        head1_inc = clamp(head1_inc, -self.max_step, self.max_step)
        head2_inc = clamp(head2_inc, -self.max_step, self.max_step)

        self.get_logger().info(
            f"x={x:.3f} y={y:.3f} z={z:.3f} | yaw_err={yaw_err:.3f} pitch_err={pitch_err:.3f} "
            f"-> inc=[{head1_inc:.3f},{head2_inc:.3f}]"
        )
        self.send_increment(head1_inc, head2_inc)


def main():
    rclpy.init()
    node = LookAtMarker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()