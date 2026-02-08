import sys
import time
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Pose
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener
from apriltag_msgs.msg import AprilTagDetectionArray
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from play_motion2_msgs.action import PlayMotion2
from moveit_wrapper_interfaces.srv import (
    AddColObj, RemoveColObj, AttachObj, DettachObj,
    GetAttachedObjs, Plan, ExecutePlans
)
from moveit_wrapper_interfaces.msg import Waypoint
from moveit_msgs.msg import Constraints, RobotState
from moveit_msgs.srv import GetPlanningScene, ApplyPlanningScene
from moveit_msgs.msg import PlanningScene, AllowedCollisionMatrix, AllowedCollisionEntry, PlanningSceneComponents
import copy

def quaternion_from_euler(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return [qx, qy, qz, qw]

def euler_from_quaternion(quaternion):
    x, y, z, w = quaternion
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return [roll, pitch, yaw]

class TestPickPlace(Node):
    def __init__(self):
        super().__init__("test_pick_place")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.add_col_client = self.create_client(AddColObj, '/moveit_wrapper/add_collision_object')
        self.remove_col_client = self.create_client(RemoveColObj, '/moveit_wrapper/remove_collision_object')
        self.attach_client = self.create_client(AttachObj, '/moveit_wrapper/attach_object')
        self.detach_client = self.create_client(DettachObj, '/moveit_wrapper/detach_object')
        self.get_attached_client = self.create_client(GetAttachedObjs, '/moveit_wrapper/get_attached_objects')
        self.plan_client = self.create_client(Plan, '/moveit_wrapper/plan')
        self.execute_client = self.create_client(ExecutePlans, '/moveit_wrapper/execute_plans')
        self.get_planning_scene_client = self.create_client(GetPlanningScene, '/get_planning_scene')
        self.apply_planning_scene_client = self.create_client(ApplyPlanningScene, '/apply_planning_scene')
        
        self.head_action_client = ActionClient(self, FollowJointTrajectory, '/head_controller/follow_joint_trajectory')
        self.play_motion2_client = ActionClient(self, PlayMotion2, '/play_motion2')
        
        self.marker_pub = self.create_publisher(MarkerArray, 'grasp_markers', 10)
        self.apriltag_sub = self.create_subscription(AprilTagDetectionArray, '/detections', self.apriltag_callback, 10)
        
        self.apriltag_detected = False
        self.latest_detection = None
        self.camera_frame = None
        self.object_pose = None
        
        self.wait_for_services()

    def wait_for_services(self):
        services = [
            (self.add_col_client, "add_collision_object"),
            (self.remove_col_client, "remove_collision_object"),
            (self.attach_client, "attach_object"),
            (self.detach_client, "detach_object"),
            (self.get_attached_client, "get_attached_objects"),
            (self.plan_client, "plan"),
            (self.execute_client, "execute_plans"),
            (self.get_planning_scene_client, "get_planning_scene"),
            (self.apply_planning_scene_client, "apply_planning_scene"),
        ]
        for client, name in services:
            while not client.wait_for_service(timeout_sec=1.0):
                pass
        self.head_action_client.wait_for_server()
        self.play_motion2_client.wait_for_server()

    def apriltag_callback(self, msg):
        if msg.detections:
            detection = msg.detections[0]
            self.latest_detection = detection
            self.camera_frame = msg.header.frame_id
            if not self.apriltag_detected:
                self.apriltag_detected = True

    def move_head_to_look_down(self, pan=0.0, tilt=-0.9):
        goal_msg = FollowJointTrajectory.Goal()
        trajectory = JointTrajectory()
        trajectory.joint_names = ['head_1_joint', 'head_2_joint']
        point = JointTrajectoryPoint()
        point.positions = [pan, tilt]
        point.velocities = [0.0, 0.0]
        point.time_from_start.sec = 2
        trajectory.points = [point]
        goal_msg.trajectory = trajectory
        future = self.head_action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if not future.done() or not future.result().accepted:
            return False
        result_future = future.result().get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=10.0)
        return True

    def wait_for_apriltag(self, timeout=10.0):
        start = time.time()
        while not self.apriltag_detected and (time.time() - start) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.apriltag_detected

    def get_apriltag_pose_in_base_link(self):
        if not self.latest_detection:
            return None
        tag_id = self.latest_detection.id
        tag_frame = f"tag36h11_{tag_id}"
        try:
            transform = self.tf_buffer.lookup_transform(
                'base_link', tag_frame, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=2.0)
            )
            return transform
        except:
            return None

    def set_gripper_object_collision_allowed(self, object_name: str, allowed: bool) -> bool:
        gripper_links = ["gripper_link", "gripper_tool_link"]
        try:
            get_request = GetPlanningScene.Request()
            get_request.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
            get_future = self.get_planning_scene_client.call_async(get_request)
            rclpy.spin_until_future_complete(self, get_future, timeout_sec=5.0)
            if not get_future.done() or get_future.result() is None:
                return False
            current_acm = get_future.result().scene.allowed_collision_matrix
            
            new_acm = copy.deepcopy(current_acm)
            
            def ensure_entry(name: str):
                if name in new_acm.entry_names:
                    return
                new_acm.entry_names.append(name)
                for row in new_acm.entry_values:
                    row.enabled.append(False)
                new_row = AllowedCollisionEntry()
                new_row.enabled = [False] * len(new_acm.entry_names)
                new_acm.entry_values.append(new_row)
            
            ensure_entry(object_name)
            for link in gripper_links:
                ensure_entry(link)
            
            obj_idx = new_acm.entry_names.index(object_name)
            for link in gripper_links:
                link_idx = new_acm.entry_names.index(link)
                new_acm.entry_values[link_idx].enabled[obj_idx] = allowed
                new_acm.entry_values[obj_idx].enabled[link_idx] = allowed
            
            apply_request = ApplyPlanningScene.Request()
            apply_request.scene = PlanningScene()
            apply_request.scene.is_diff = True
            apply_request.scene.allowed_collision_matrix = new_acm
            apply_future = self.apply_planning_scene_client.call_async(apply_request)
            rclpy.spin_until_future_complete(self, apply_future, timeout_sec=5.0)
            return apply_future.done() and apply_future.result().success
        except:
            return False

    def create_collision_objects(self, apriltag_transform):
        tag_x = apriltag_transform.transform.translation.x
        tag_y = apriltag_transform.transform.translation.y
        tag_z = apriltag_transform.transform.translation.z
        tag_quat = [
            apriltag_transform.transform.rotation.x,
            apriltag_transform.transform.rotation.y,
            apriltag_transform.transform.rotation.z,
            apriltag_transform.transform.rotation.w
        ]
        
        object_x = tag_x + 0.028
        object_y = tag_y + 0.01
        object_z = tag_z + 0.08
        
        self.object_pose = Pose()
        self.object_pose.position.x = object_x
        self.object_pose.position.y = object_y
        self.object_pose.position.z = object_z
        self.object_pose.orientation.x = tag_quat[0]
        self.object_pose.orientation.y = tag_quat[1]
        self.object_pose.orientation.z = tag_quat[2]
        self.object_pose.orientation.w = tag_quat[3]
        
        obj_width = 0.04
        obj_depth = 0.06
        obj_height = 0.15
        
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "object"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose = self.object_pose
        marker.scale.x = obj_width
        marker.scale.y = obj_depth
        marker.scale.z = obj_height
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.8
        
        request = AddColObj.Request()
        request.objects = [marker]
        future = self.add_col_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        response = future.result()
        return response and response.success

    def create_grasp_waypoints(self, object_pose):
        obj_x = object_pose.position.x
        obj_y = object_pose.position.y
        obj_z = object_pose.position.z
        
        approach_yaw = math.atan2(obj_y, obj_x)
        gripper_quat = quaternion_from_euler(0.0, 0.0, approach_yaw)
        
        def make_pose(x, y, z, set_orientation=True):
            pose = Pose()
            pose.position.x = float(x)
            pose.position.y = float(y)
            pose.position.z = float(z)
            if set_orientation:
                pose.orientation.x = gripper_quat[0]
                pose.orientation.y = gripper_quat[1]
                pose.orientation.z = gripper_quat[2]
                pose.orientation.w = gripper_quat[3]
            else:
                pose.orientation.x = 0.0
                pose.orientation.y = 0.0
                pose.orientation.z = 0.0
                pose.orientation.w = 1.0
            return pose
        
        pre_dist = 0.15
        pre_x = obj_x - pre_dist * math.cos(approach_yaw)
        pre_y = obj_y - pre_dist * math.sin(approach_yaw)
        pre_z = obj_z + 0.02
        
        grasp_dist = 0.02
        g_x = obj_x - grasp_dist * math.cos(approach_yaw)
        g_y = obj_y - grasp_dist * math.sin(approach_yaw)
        g_z = obj_z + 0.02
        
        l_x, l_y, l_z = g_x, g_y, g_z + 0.05
        p_x, p_y, p_z = g_x, g_y, g_z + 0.01
        retreat_x, retreat_y, retreat_z = pre_x, pre_y, l_z
        
        pregrasp_pose = make_pose(pre_x, pre_y, pre_z, set_orientation=False)
        grasp_pose = make_pose(g_x, g_y, g_z, set_orientation=True)
        lift_pose = make_pose(l_x, l_y, l_z, set_orientation=True)
        place_pose = make_pose(p_x, p_y, p_z, set_orientation=True)
        retreat_pose = make_pose(retreat_x, retreat_y, retreat_z, set_orientation=True)
        
        def pose_to_waypoint(pose):
            waypoint = Waypoint()
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "base_link"
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            pose_stamped.pose = pose
            waypoint.pose = pose_stamped
            return waypoint
        
        return (
            pose_to_waypoint(pregrasp_pose),
            pose_to_waypoint(grasp_pose),
            pose_to_waypoint(lift_pose),
            pose_to_waypoint(place_pose),
            pose_to_waypoint(retreat_pose)
        )

    def clear_planning_scene(self):
        detach_req = DettachObj.Request()
        detach_req.move_group = "arm_torso"
        detach_req.object_names = []
        detach_future = self.detach_client.call_async(detach_req)
        rclpy.spin_until_future_complete(self, detach_future, timeout_sec=5.0)
        
        remove_req = RemoveColObj.Request()
        remove_req.object_names = []
        remove_future = self.remove_col_client.call_async(remove_req)
        rclpy.spin_until_future_complete(self, remove_future, timeout_sec=5.0)
        return True

    def control_gripper(self, command="open"):
        goal = PlayMotion2.Goal()
        goal.motion_name = command
        goal.skip_planning = False
        future = self.play_motion2_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if not future.done() or not future.result().accepted:
            return False
        result_future = future.result().get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=5.0)
        return result_future.done() and result_future.result().status == 4

    def plan_and_execute(self, waypoint, description=""):
        request = Plan.Request()
        request.waypoints = [waypoint]
        request.path_constraints = Constraints()
        request.send_partial = True
        request.use_start_state = False
        request.start_state = RobotState()
        request.move_group = "arm_torso"
        future = self.plan_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=15.0)
        response = future.result()
        if not response or not response.success:
            return False
        
        exec_request = ExecutePlans.Request()
        exec_request.plans = response.plans
        exec_request.move_group = "arm_torso"
        exec_future = self.execute_client.call_async(exec_request)
        rclpy.spin_until_future_complete(self, exec_future, timeout_sec=30.0)
        exec_response = exec_future.result()
        return exec_response and exec_response.success

    def attach_object(self, object_name="object"):
        request = AttachObj.Request()
        request.move_group = "arm_torso"
        request.object_names = [object_name]
        future = self.attach_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        response = future.result()
        return response and response.success

    def detach_object(self, object_name="object"):
        request = DettachObj.Request()
        request.move_group = "arm_torso"
        request.object_names = [object_name]
        future = self.detach_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        response = future.result()
        return response and response.success

    def run_pick_and_place(self):
        if not self.clear_planning_scene():
            return False
        time.sleep(2.0)
        
        if not self.move_head_to_look_down():
            return False
        time.sleep(2.0)
        
        if not self.wait_for_apriltag():
            return False
        time.sleep(2.0)
        
        apriltag_transform = self.get_apriltag_pose_in_base_link()
        if not apriltag_transform:
            return False
        
        if not self.create_collision_objects(apriltag_transform):
            return False
        time.sleep(3.0)
        
        pregrasp_wp, grasp_wp, lift_wp, place_wp, retreat_wp = self.create_grasp_waypoints(self.object_pose)
        time.sleep(1.0)
        
        if not self.control_gripper("open"):
            pass
        time.sleep(1.0)
        
        if not self.plan_and_execute(pregrasp_wp, "pregrasp"):
            return False
        time.sleep(1.0)
        
        self.set_gripper_object_collision_allowed("object", allowed=True)
        time.sleep(0.5)
        
        if not self.plan_and_execute(grasp_wp, "grasp"):
            return False
        time.sleep(1.0)
        
        if not self.attach_object("object"):
            return False
        time.sleep(0.5)
        
        if not self.control_gripper("close"):
            return False
        time.sleep(2.0)
        
        if not self.plan_and_execute(lift_wp, "lift"):
            return False
        time.sleep(1.0)
        
        if not self.plan_and_execute(retreat_wp, "retreat"):
            return False
        time.sleep(1.0)
        
        if not self.plan_and_execute(place_wp, "place"):
            return False
        time.sleep(1.0)
        
        if not self.control_gripper("open"):
            return False
        time.sleep(1.0)
        
        if not self.detach_object("object"):
            return False
        time.sleep(1.0)
        
        return True

def main():
    rclpy.init()
    node = TestPickPlace()
    try:
        success = node.run_pick_and_place()
        sys.exit(0 if success else 1)
    except:
        sys.exit(1)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()