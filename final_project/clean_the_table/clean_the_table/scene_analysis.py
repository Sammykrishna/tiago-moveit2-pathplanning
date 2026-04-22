#!/usr/bin/env python3
import sys
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
from visualization_msgs.msg import Marker
from apriltag_msgs.msg import AprilTagDetectionArray
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_wrapper_interfaces.srv import AddColObj, RemoveColObj

class SceneAnalysis(Node):
    def __init__(self):
        super().__init__("scene_analysis")
        self.get_logger().info("scene_analysis node started")
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.add_col_client = self.create_client(AddColObj, "/moveit_wrapper/add_collision_object")
        self.remove_col_client = self.create_client(RemoveColObj, "/moveit_wrapper/remove_collision_object")
        self.head_action_client = ActionClient(self, FollowJointTrajectory, '/head_controller/follow_joint_trajectory')
        self.apriltag_sub = self.create_subscription(AprilTagDetectionArray, '/detections', self.apriltag_callback, 10)
        
        # Use TRANSIENT_LOCAL so markers are available to late subscribers
        from rclpy.qos import QoSProfile, DurabilityPolicy
        marker_qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.marker_pub = self.create_publisher(Marker, 'move_markers', marker_qos)
        
        self.tag_detections = {}
        self.camera_frame = None
        
        # Store markers for continuous publishing
        self.stored_markers = []
        
        # Create timer to republish markers every second
        self.marker_timer = self.create_timer(1.0, self.publish_stored_markers)
        
        self.wait_for_services()
    
    def publish_stored_markers(self):
        """Republish stored markers continuously"""
        if self.stored_markers:
            for marker in self.stored_markers:
                marker.header.stamp = self.get_clock().now().to_msg()
                self.marker_pub.publish(marker)
            # Log only once when markers start being published
            if not hasattr(self, '_markers_logged'):
                self.get_logger().info(f"Publishing {len(self.stored_markers)} markers continuously on /move_markers")
                self._markers_logged = True

    # Wait for required ROS2 services and action servers to become available
    def wait_for_services(self):
        while not self.add_col_client.wait_for_service(timeout_sec=1.0):
            pass
        while not self.remove_col_client.wait_for_service(timeout_sec=1.0):
            pass
        self.head_action_client.wait_for_server()

    def apriltag_callback(self, msg):
        if msg.detections:
            self.camera_frame = msg.header.frame_id
            for detection in msg.detections:
                self.tag_detections[detection.id] = detection

    # Command head joints to look downward using trajectory action interface
    def move_head_to_look_down(self, pan=0.0, tilt=-0.6):
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
        
        if future.result() is None or not future.result().accepted:
            return False
        
        result_future = future.result().get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=10.0)
        return True

    # Wait until all required AprilTag IDs are detected within timeout period
    def wait_for_apriltags(self, required_ids, timeout_sec=30.0):
        start_time = time.time()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if all(tag_id in self.tag_detections for tag_id in required_ids):
                return True
            if time.time() - start_time > timeout_sec:
                return False
        return False

    # Estimate 3D tag pose from pixel coordinates using fixed distance assumption and transform to base_link frame
    def get_apriltag_pose_in_base_link(self, tag_id):
        if tag_id not in self.tag_detections or not self.camera_frame:
            return None
        try:
            detection = self.tag_detections[tag_id]
            centre_u = detection.centre.x
            centre_v = detection.centre.y
            
            image_width = 640.0
            image_height = 480.0
            focal_length = 554.0
            estimated_distance = 0.7
            
            cx = image_width / 2.0
            cy = image_height / 2.0
            x_norm = (centre_u - cx) / focal_length
            y_norm = (centre_v - cy) / focal_length
            
            tag_pose_camera = PoseStamped()
            tag_pose_camera.header.frame_id = self.camera_frame
            tag_pose_camera.header.stamp = rclpy.time.Time().to_msg()
            tag_pose_camera.pose.position.x = x_norm * estimated_distance
            tag_pose_camera.pose.position.y = y_norm * estimated_distance
            tag_pose_camera.pose.position.z = estimated_distance
            tag_pose_camera.pose.orientation.w = 1.0
            
            tag_pose_base = self.tf_buffer.transform(
                tag_pose_camera,
                "base_link",
                timeout=rclpy.duration.Duration(seconds=2.0)
            )
            return tag_pose_base.pose
        except Exception as e:
            return None

    # Create a visualization marker positioned relative to an AprilTag's pose with specified offsets
    def create_marker_relative_to_apriltag(self, tag_id, marker_type, offset_x, offset_y, offset_z, scale_x, scale_y, scale_z, ns="collision_object"):
        if tag_id not in self.tag_detections:
            return None
        tag_pose = self.get_apriltag_pose_in_base_link(tag_id)
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
        marker.pose.orientation.w = 1.0
        marker.scale.x = scale_x
        marker.scale.y = scale_y
        marker.scale.z = scale_z
        marker.color.g = 1.0
        marker.color.a = 0.8
        return marker

    # Generate side-grasp approach markers (grasp + pregrasp positions) for manipulation planning
    def create_grasp_markers(self, object_pose, object_id):
        markers = []
        
        grasp_marker = Marker()
        grasp_marker.header.frame_id = "base_link"
        grasp_marker.header.stamp = self.get_clock().now().to_msg()
        grasp_marker.ns = f"grasp_object_{object_id}"
        grasp_marker.id = object_id * 2
        grasp_marker.type = Marker.ARROW
        grasp_marker.action = Marker.ADD
        grasp_marker.pose.position.x = object_pose.position.x - 0.10
        grasp_marker.pose.position.y = object_pose.position.y
        grasp_marker.pose.position.z = object_pose.position.z
        grasp_marker.pose.orientation.w = 1.0
        grasp_marker.scale.x = 0.1
        grasp_marker.scale.y = 0.01
        grasp_marker.scale.z = 0.01
        grasp_marker.color.g = 1.0
        grasp_marker.color.a = 1.0
        grasp_marker.lifetime.sec = 0  # Fixed: use .sec instead of Duration().to_msg()
        markers.append(grasp_marker)
        
        pregrasp_marker = Marker()
        pregrasp_marker.header.frame_id = "base_link"
        pregrasp_marker.header.stamp = self.get_clock().now().to_msg()
        pregrasp_marker.ns = f"pregrasp_object_{object_id}"
        pregrasp_marker.id = object_id * 2 + 1
        pregrasp_marker.type = Marker.ARROW
        pregrasp_marker.action = Marker.ADD
        pregrasp_marker.pose.position.x = object_pose.position.x - 0.25
        pregrasp_marker.pose.position.y = object_pose.position.y
        pregrasp_marker.pose.position.z = object_pose.position.z
        pregrasp_marker.pose.orientation.w = 1.0
        pregrasp_marker.scale.x = 0.15
        pregrasp_marker.scale.y = 0.01
        pregrasp_marker.scale.z = 0.01
        pregrasp_marker.color.b = 1.0
        pregrasp_marker.color.a = 1.0
        pregrasp_marker.lifetime.sec = 0  # Fixed: use .sec instead of Duration().to_msg()
        markers.append(pregrasp_marker)
        
        return markers

    # Publish visualization markers to RViz with small delay between publications
    def publish_markers(self, markers):
        for marker in markers:
            # Store for continuous republishing
            self.stored_markers.append(marker)
            # Publish immediately
            self.marker_pub.publish(marker)
            time.sleep(0.01)

    # Add collision objects to MoveIt planning scene with timeout handling
    def add_collision_objects(self, markers):
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
        except Exception as e:
            return False

    # Create and publish side-grasp approach markers with proper orientation for gripper alignment
    def create_and_publish_grasp_markers(self, object_pose, object_id):
        grasp_marker = Marker()
        grasp_marker.header.frame_id = "base_link"
        grasp_marker.header.stamp = self.get_clock().now().to_msg()
        grasp_marker.ns = f"grasp_object_{object_id}"
        grasp_marker.id = object_id * 2
        grasp_marker.type = Marker.ARROW
        grasp_marker.action = Marker.ADD
        # Grasp position: approach from -X direction (robot approaches from behind the object)
        grasp_marker.pose.position.x = object_pose.position.x - 0.10  # 10cm behind object in X
        grasp_marker.pose.position.y = object_pose.position.y
        grasp_marker.pose.position.z = object_pose.position.z
        # Orientation: pointing in +X direction (no rotation, identity quaternion)
        grasp_marker.pose.orientation.x = 0.0
        grasp_marker.pose.orientation.y = 0.0
        grasp_marker.pose.orientation.z = 0.0
        grasp_marker.pose.orientation.w = 1.0
        grasp_marker.scale.x = 0.1  # Arrow length
        grasp_marker.scale.y = 0.01
        grasp_marker.scale.z = 0.01
        grasp_marker.color.r = 1.0  # RED for grasp
        grasp_marker.color.g = 0.0
        grasp_marker.color.b = 0.0
        grasp_marker.color.a = 1.0
        grasp_marker.lifetime.sec = 0
        
        pregrasp_marker = Marker()
        pregrasp_marker.header.frame_id = "base_link"
        pregrasp_marker.header.stamp = self.get_clock().now().to_msg()
        pregrasp_marker.ns = f"pregrasp_object_{object_id}"
        pregrasp_marker.id = object_id * 2 + 1
        pregrasp_marker.type = Marker.ARROW
        pregrasp_marker.action = Marker.ADD
        # Pre-grasp position: further back in -X direction (15cm behind object)
        pregrasp_marker.pose.position.x = object_pose.position.x - 0.15  # 15cm behind object in X
        pregrasp_marker.pose.position.y = object_pose.position.y
        pregrasp_marker.pose.position.z = object_pose.position.z
        # Orientation: pointing in +X direction (no rotation, identity quaternion)
        pregrasp_marker.pose.orientation.x = 0.0
        pregrasp_marker.pose.orientation.y = 0.0
        pregrasp_marker.pose.orientation.z = 0.0
        pregrasp_marker.pose.orientation.w = 1.0
        pregrasp_marker.scale.x = 0.1  # Arrow length
        pregrasp_marker.scale.y = 0.01
        pregrasp_marker.scale.z = 0.01
        pregrasp_marker.color.r = 0.0
        pregrasp_marker.color.g = 1.0  # GREEN for pregrasp
        pregrasp_marker.color.b = 0.0
        pregrasp_marker.color.a = 1.0
        pregrasp_marker.lifetime.sec = 0
        
        # Store markers for continuous republishing
        self.stored_markers.append(grasp_marker)
        self.stored_markers.append(pregrasp_marker)
        
        # Publish immediately
        self.marker_pub.publish(grasp_marker)
        self.marker_pub.publish(pregrasp_marker)

    # Remove all collision objects from MoveIt planning scene
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
        except Exception as e:
            return False

    # Main test sequence: head movement, tag detection, scene setup, and collision object registration
    def run_test(self):
        self.get_logger().info("=" * 60)
        self.get_logger().info("STARTING RUN_TEST")
        self.get_logger().info("=" * 60)
        
        self.get_logger().info("[STEP 1] Moving head to look down...")
        if not self.move_head_to_look_down(pan=0.0, tilt=-0.9):
            self.get_logger().error("❌ Failed to move head!")
            return False
        self.get_logger().info("✓ Head moved")
        
        self.get_logger().info("[STEP 2] Waiting 5 seconds for stabilization...")
        time.sleep(5.0)
        
        self.get_logger().info("[STEP 3] Waiting for AprilTags...")
        required_ids = [1, 2, 3, 11]
        if not self.wait_for_apriltags(required_ids, timeout_sec=30.0):
            self.get_logger().error("❌ Failed to detect all AprilTags!")
            self.get_logger().error(f"   Detected tags: {list(self.tag_detections.keys())}")
            return False
        self.get_logger().info(f"✓ All tags detected: {list(self.tag_detections.keys())}")
        
        self.get_logger().info("[STEP 4] Waiting 3 seconds for detection stabilization...")
        time.sleep(3.0)
        
        self.get_logger().info("[STEP 5] Clearing collision objects...")
        self.clear_collision_objects()
        
        self.get_logger().info("[STEP 6] Creating collision markers...")
        markers = []
        
        self.get_logger().info("  Creating table marker...")
        table_marker = self.create_marker_relative_to_apriltag(
            tag_id=11,
            marker_type=Marker.CYLINDER,
            offset_x=0.3,
            offset_y=-0.0,
            offset_z=-0.065,
            scale_x=0.52,
            scale_y=0.52,
            scale_z=0.04,
            ns="table"
        )
        if table_marker is None:
            self.get_logger().error("❌ Failed to create table marker!")
            return False
        markers.append(table_marker)
        self.get_logger().info("  ✓ Table marker created")
        
        self.get_logger().info("  Creating object 1 marker...")
        object1_marker = self.create_marker_relative_to_apriltag(
            tag_id=1,
            marker_type=Marker.CUBE,
            offset_x=0.14,
            offset_y=0.02,
            offset_z=0.02,
            scale_x=0.04,
            scale_y=0.06,
            scale_z=0.15,
            ns="object_1"
        )
        if object1_marker is None:
            self.get_logger().error("❌ Failed to create object 1 marker!")
            return False
        markers.append(object1_marker)
        self.create_and_publish_grasp_markers(object1_marker.pose, object_id=1)
        self.get_logger().info("  ✓ Object 1 marker + grasp markers created")
        
        self.get_logger().info("  Creating object 2 marker...")
        object2_marker = self.create_marker_relative_to_apriltag(
            tag_id=2,
            marker_type=Marker.CUBE,
            offset_x=0.2,
            offset_y=0.013,
            offset_z=-0.018,
            scale_x=0.08,
            scale_y=0.06,
            scale_z=0.15,
            ns="object_2"
        )
        if object2_marker is None:
            self.get_logger().error("❌ Failed to create object 2 marker!")
            return False
        markers.append(object2_marker)
        self.create_and_publish_grasp_markers(object2_marker.pose, object_id=2)
        self.get_logger().info("  ✓ Object 2 marker + grasp markers created")
        
        self.get_logger().info("  Creating object 3 marker...")
        object3_marker = self.create_marker_relative_to_apriltag(
            tag_id=3,
            marker_type=Marker.CUBE,
            offset_x=0.15,
            offset_y=-0.013,
            offset_z=0.0,
            scale_x=0.08,
            scale_y=0.06,
            scale_z=0.15,
            ns="object_3"
        )
        if object3_marker is None:
            self.get_logger().error("❌ Failed to create object 3 marker!")
            return False
        markers.append(object3_marker)
        self.create_and_publish_grasp_markers(object3_marker.pose, object_id=3)
        self.get_logger().info("  ✓ Object 3 marker + grasp markers created")
        
        self.get_logger().info("[STEP 7] Adding collision objects to MoveIt...")
        if not self.add_collision_objects(markers):
            self.get_logger().error("❌ Failed to add collision objects!")
            return False
        self.get_logger().info("✓ Collision objects added")
        
        # Note: Grasp markers already created and published by create_and_publish_grasp_markers()
        # No need to duplicate them here
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("✅ Scene setup complete")
        self.get_logger().info(f"✅ Total markers stored: {len(self.stored_markers)} (publishing continuously)")
        self.get_logger().info("=" * 60)
        return True

def main():
    rclpy.init()
    node = SceneAnalysis()
    success = node.run_test()
    if success:
        rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    return 0 if success else 1

if __name__ == "__main__":
    main()