#!/usr/bin/env python3
import sys
import time
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
from visualization_msgs.msg import Marker
from apriltag_msgs.msg import AprilTagDetectionArray
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_wrapper_interfaces.srv import AddColObj, RemoveColObj

class CollisionObjectsTest(Node):
    def __init__(self):
        super().__init__("test_collision_objects")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.add_col_client = self.create_client(AddColObj, "/moveit_wrapper/add_collision_object")
        self.remove_col_client = self.create_client(RemoveColObj, "/moveit_wrapper/remove_collision_object")
        
        self.head_action_client = ActionClient(self, FollowJointTrajectory, '/head_controller/follow_joint_trajectory')
        
        self.apriltag_sub = self.create_subscription(AprilTagDetectionArray, '/detections', self.apriltag_callback, 10)
        
        self.apriltag_detected = False
        self.apriltag_frame = None
        self.apriltag_id = None
        self.latest_detection = None
        self.camera_frame = None
        
        self.wait_for_services()

    def wait_for_services(self):
        while not self.add_col_client.wait_for_service(timeout_sec=1.0):
            pass
        while not self.remove_col_client.wait_for_service(timeout_sec=1.0):
            pass
        self.head_action_client.wait_for_server()

    def apriltag_callback(self, msg):
        if msg.detections:
            detection = msg.detections[0]
            self.apriltag_id = detection.id
            self.apriltag_frame = f"tag_{detection.id}"
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
        if future.result() is None:
            return False
        goal_handle = future.result()
        if not goal_handle.accepted:
            return False
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=10.0)
        return True

    def wait_for_apriltag(self, timeout_sec=30.0):
        start_time = time.time()
        while rclpy.ok() and not self.apriltag_detected:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start_time > timeout_sec:
                return False
        return True

    def get_apriltag_pose_in_base_link(self):
        if not self.latest_detection or not self.camera_frame:
            return None
        try:
            estimated_distance = 0.5
            tag_pose_camera = PoseStamped()
            tag_pose_camera.header.frame_id = self.camera_frame
            tag_pose_camera.header.stamp = rclpy.time.Time().to_msg()
            tag_pose_camera.pose.position.x = 0.0
            tag_pose_camera.pose.position.y = 0.0
            tag_pose_camera.pose.position.z = estimated_distance
            tag_pose_camera.pose.orientation.x = 0.0
            tag_pose_camera.pose.orientation.y = 0.0
            tag_pose_camera.pose.orientation.z = 0.0
            tag_pose_camera.pose.orientation.w = 1.0
            
            tag_pose_base = self.tf_buffer.transform(
                tag_pose_camera,
                "base_link",
                timeout=rclpy.duration.Duration(seconds=2.0)
            )
            return tag_pose_base.pose
        except:
            return None

    def create_marker_relative_to_apriltag(self, marker_type, offset_x, offset_y, offset_z, scale_x, scale_y, scale_z, ns="collision_object"):
        if not self.apriltag_detected:
            return None
        tag_pose = self.get_apriltag_pose_in_base_link()
        if tag_pose is None:
            return None
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = ns
        marker.id = 0
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.position.x = tag_pose.position.x + offset_x
        marker.pose.position.y = tag_pose.position.y + offset_y
        marker.pose.position.z = tag_pose.position.z + offset_z
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = scale_x
        marker.scale.y = scale_y
        marker.scale.z = scale_z
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.8
        return marker

    def add_collision_objects(self, markers):
        if not markers:
            return False
        try:
            req = AddColObj.Request()
            req.objects = markers
            future = self.add_col_client.call_async(req)
            start_time = time.time()
            while rclpy.ok() and not future.done():
                rclpy.spin_once(self, timeout_sec=0.1)
                if time.time() - start_time > 10.0:
                    return False
            response = future.result()
            if hasattr(response, 'success') and not response.success:
                return False
            return True
        except:
            return False

    def clear_collision_objects(self):
        try:
            req = RemoveColObj.Request()
            req.object_names = []
            future = self.remove_col_client.call_async(req)
            start_time = time.time()
            while rclpy.ok() and not future.done():
                rclpy.spin_once(self, timeout_sec=0.1)
                if time.time() - start_time > 10.0:
                    return False
            return True
        except:
            return False

    def run_test(self):
        if not self.move_head_to_look_down(pan=0.0, tilt=-0.9):
            return False
        time.sleep(3.0)
        
        if not self.wait_for_apriltag(timeout_sec=30.0):
            return False
        time.sleep(2.0)
        
        self.clear_collision_objects()
        
        table_marker = self.create_marker_relative_to_apriltag(
            marker_type=Marker.CYLINDER,
            offset_x=0.3,
            offset_y=0.0,
            offset_z=-0.10,
            scale_x=0.65,
            scale_y=0.65,
            scale_z=0.05,
            ns="table"
        )
        if table_marker is None:
            return False
        
        object_marker = self.create_marker_relative_to_apriltag(
            marker_type=Marker.CUBE,
            offset_x=0.2,
            offset_y=0.0,
            offset_z=0.04,
            scale_x=0.06,
            scale_y=0.06,
            scale_z=0.08,
            ns="object"
        )
        if object_marker is None:
            return False
        
        markers = [table_marker, object_marker]
        if not self.add_collision_objects(markers):
            return False
        
        return True

def main():
    rclpy.init()
    node = CollisionObjectsTest()
    try:
        success = node.run_test()
        if success:
            rclpy.spin(node)
        else:
            sys.exit(1)
    except KeyboardInterrupt:
        pass
    except:
        sys.exit(1)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()