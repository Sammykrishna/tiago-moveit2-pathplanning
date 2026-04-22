#!/usr/bin/env python3

import sys
import time
import copy
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from moveit_wrapper_interfaces.msg import Waypoint
from moveit_wrapper_interfaces.srv import (
    AttachObj, 
    DettachObj, 
    ExecutePlans, 
    Plan,
    GotoNamedTarget
)
from play_motion2_msgs.action import PlayMotion2
from builtin_interfaces.msg import Duration
from moveit_msgs.msg import Constraints, RobotState
from gazebo_msgs.srv import DeleteEntity


class PickAndCarry(Node):
    def __init__(self):
        super().__init__("pick_and_carry")
        self.get_logger().info("🚀 Initializing pick_and_carry node...")
        
        # MoveIt2 wrapper service clients
        self.plan_client = self.create_client(Plan, "/moveit_wrapper/plan")
        self.execute_client = self.create_client(ExecutePlans, "/moveit_wrapper/execute_plans")
        self.goto_named_target_client = self.create_client(GotoNamedTarget, "/moveit_wrapper/goto_named_target")
        self.attach_col_client = self.create_client(AttachObj, "/moveit_wrapper/attach_object")
        self.detach_col_client = self.create_client(DettachObj, "/moveit_wrapper/detach_object")
        
        # Gazebo service client for deleting entities
        self.delete_entity_client = self.create_client(DeleteEntity, '/delete_entity')
        
        # PlayMotion2 action client for gripper control
        self.play_motion_client = ActionClient(self, PlayMotion2, '/play_motion2')
        
        # Subscribe to grasp/pregrasp markers with TRANSIENT_LOCAL to receive cached messages
        from rclpy.qos import QoSProfile, DurabilityPolicy
        marker_qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.marker_sub = self.create_subscription(
            Marker, 
            '/move_markers', 
            self.marker_callback, 
            marker_qos
        )
        
        # Storage for grasp and pregrasp poses for each object
        self.grasp_poses = {}      # {object_id: PoseStamped}
        self.pregrasp_poses = {}   # {object_id: PoseStamped}
        
        # Track which objects we've already logged to reduce spam
        self.logged_objects = set()
        
        # Map object IDs to Gazebo entity names (e.g., object_1 -> box1)
        self.gazebo_entity_names = {
            1: "box1",
            2: "box2", 
            3: "box3"
        }
        
        self.get_logger().info("⏳ Waiting for services...")
        self.wait_for_services()
        self.get_logger().info("✓ All services ready!")
        
    def wait_for_services(self):
        """Wait for all required services and action servers"""
        services = [
            (self.plan_client, "/moveit_wrapper/plan"),
            (self.execute_client, "/moveit_wrapper/execute_plans"),
            (self.goto_named_target_client, "/moveit_wrapper/goto_named_target"),
            (self.attach_col_client, "/moveit_wrapper/attach_object"),
            (self.detach_col_client, "/moveit_wrapper/detach_object"),
            (self.delete_entity_client, "/delete_entity"),
        ]
        
        for client, service_name in services:
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f"  Waiting for {service_name}...")
        
        self.get_logger().info("  Waiting for PlayMotion2 action server...")
        self.play_motion_client.wait_for_server()
        
    def _wait_future(self, future, timeout_sec: float) -> bool:
        """Wait for a future to complete with timeout"""
        start = time.time()
        while rclpy.ok() and not future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start > timeout_sec:
                return False
        return future.done()
    
    def compute_pregrasp_from_grasp(self, grasp_pose, offset=0.05):
        """
        Compute pre-grasp pose from grasp pose by offsetting along -X direction
        
        Args:
            grasp_pose: PoseStamped - the grasp pose
            offset: float - distance to offset in meters (default: 5cm)
        
        Returns:
            PoseStamped - the computed pre-grasp pose
        """
        pregrasp_pose = copy.deepcopy(grasp_pose)
        
        # Since grasp approach is along +X axis, pre-grasp is offset in -X direction
        pregrasp_pose.pose.position.x -= offset
        
        # Orientation remains the same as grasp (both pointing in +X direction)
        return pregrasp_pose
    
    def compute_lift_pose(self, grasp_pose, z_offset=0.1, x_offset=0.0):
        """
        Compute lift pose from grasp pose by lifting in Z direction
        This is used AFTER grasping to lift the object clear of the table
        
        Args:
            grasp_pose: PoseStamped - the grasp pose
            z_offset: float - height to lift in meters (default: 15cm)
            x_offset: float - optional X offset (default: 0cm, stays at grasp X position)
        
        Returns:
            PoseStamped - the computed lift pose
        """
        lift_pose = copy.deepcopy(grasp_pose)
        
        # Lift up in Z to clear the table/obstacles
        lift_pose.pose.position.z += z_offset
        
        # Optional: Move back in X (usually keep same X as grasp for vertical lift)
        lift_pose.pose.position.x -= x_offset
        
        # Orientation remains the same as grasp
        return lift_pose
    
    def compute_adjusted_grasp_pose(self, grasp_pose, x_offset=0.01, y_offset=0.0, z_offset=0.0):
        """
        Compute adjusted grasp pose from the original grasp pose
        Useful for fine-tuning the grasp position relative to the AprilTag detection
        
        Args:
            grasp_pose: PoseStamped - the original grasp pose from scene_analysis
            x_offset: float - X-axis adjustment in meters (default: 0cm)
            y_offset: float - Y-axis adjustment in meters (default: 0cm)
            z_offset: float - Z-axis adjustment in meters (default: 0cm)
        
        Returns:
            PoseStamped - the adjusted grasp pose
        
        Example:
            # Move grasp 2cm forward in X, 1cm up in Z
            adjusted_grasp = compute_adjusted_grasp_pose(grasp_pose, x_offset=0.02, z_offset=0.01)
        """
        adjusted_grasp = copy.deepcopy(grasp_pose)
        
        # Apply offsets
        adjusted_grasp.pose.position.x += x_offset
        adjusted_grasp.pose.position.y += y_offset
        adjusted_grasp.pose.position.z += z_offset
        
        # Orientation remains the same
        return adjusted_grasp
    
    def marker_callback(self, msg):
        """Receive and store grasp/pregrasp markers"""
        # Extract object ID from namespace (e.g., "grasp_object_1" or "pregrasp_object_2")
        if "grasp_object_" in msg.ns:
            object_id = int(msg.ns.split("_")[-1])
            
            # Convert marker pose to PoseStamped
            pose = PoseStamped()
            pose.header.frame_id = msg.header.frame_id
            pose.header.stamp = msg.header.stamp
            pose.pose = msg.pose
            
            self.grasp_poses[object_id] = pose
            
            # Only log the first time we receive this object
            if object_id not in self.logged_objects:
                self.get_logger().info(f"📍 Received GRASP pose for object {object_id}")
                self.logged_objects.add(object_id)
            
            # Automatically compute pre-grasp from grasp pose
            pregrasp_pose = self.compute_pregrasp_from_grasp(pose, offset=0.05)
            self.pregrasp_poses[object_id] = pregrasp_pose
            
            # Only log computation once per object
            if f"pregrasp_{object_id}" not in self.logged_objects:
                self.get_logger().info(f"🔧 Computed PREGRASP pose for object {object_id} (offset: -5cm in X)")
                self.logged_objects.add(f"pregrasp_{object_id}")
            
        elif "pregrasp_object_" in msg.ns:
            # Still keep this in case scene_analysis starts publishing pregrasp markers again
            object_id = int(msg.ns.split("_")[-1])
            
            pose = PoseStamped()
            pose.header.frame_id = msg.header.frame_id
            pose.header.stamp = msg.header.stamp
            pose.pose = msg.pose
            
            # Only override if we don't have a computed one, or if this is explicitly published
            self.pregrasp_poses[object_id] = pose
            
            # Only log once per object
            if f"pregrasp_marker_{object_id}" not in self.logged_objects:
                self.get_logger().info(f"📍 Received PREGRASP pose for object {object_id} (from marker)")
                self.logged_objects.add(f"pregrasp_marker_{object_id}")
    
    def open_gripper(self):
        """Open the gripper using PlayMotion2"""
        self.get_logger().info("✋ Opening gripper...")
        
        goal = PlayMotion2.Goal()
        goal.motion_name = "open"
        goal.skip_planning = False
        
        future = self.play_motion_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("❌ Gripper open goal rejected!")
            return False
        
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        self.get_logger().info("✓ Gripper opened")
        return True
    
    def close_gripper(self):
        """Close the gripper using PlayMotion2"""
        self.get_logger().info("✊ Closing gripper...")
        
        goal = PlayMotion2.Goal()
        goal.motion_name = "close"
        goal.skip_planning = False
        
        future = self.play_motion_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("❌ Gripper close goal rejected!")
            return False
        
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        self.get_logger().info("✓ Gripper closed")
        return True
    
    def plan_to_pose(self, target_pose, description=""):
        """Plan motion to a target pose"""
        if description:
            self.get_logger().info(f"📝 Planning motion to {description}...")
        
        # Create waypoint
        waypoint = Waypoint()
        waypoint.pose = target_pose
        
        # Create planning request
        request = Plan.Request()
        request.waypoints = [waypoint]
        request.path_constraints = Constraints()
        request.send_partial = True
        request.use_start_state = False
        request.start_state = RobotState()
        request.move_group = "arm_torso"
        
        try:
            future = self.plan_client.call_async(request)
            if not self._wait_future(future, 15.0):
                self.get_logger().error("❌ Planning timeout!")
                return None
            
            response = future.result()
            if response.success:
                self.get_logger().info(f"✓ Planning successful! {len(response.plans)} trajectory segments")
                return response
            else:
                self.get_logger().error(f"❌ Planning failed: {response.message}")
                return None
                
        except Exception as e:
            self.get_logger().error(f"❌ Planning service failed: {e}")
            return None
    
    def execute_plan(self, plans):
        """Execute planned trajectory"""
        self.get_logger().info("🤖 Executing trajectory...")
        
        request = ExecutePlans.Request()
        request.plans = plans
        request.move_group = "arm_torso"
        
        try:
            future = self.execute_client.call_async(request)
            if not self._wait_future(future, 30.0):
                self.get_logger().error("❌ Execution timeout!")
                return False
            
            response = future.result()
            if response.success:
                self.get_logger().info("✓ Execution successful!")
                return True
            else:
                self.get_logger().error(f"❌ Execution failed: {response.message}")
                return False
                
        except Exception as e:
            self.get_logger().error(f"❌ Execution service failed: {e}")
            return False
    
    def attach_object(self, object_name):
        """Attach object to gripper"""
        self.get_logger().info(f"🔗 Attaching object: {object_name}")
        
        try:
            request = AttachObj.Request()
            request.move_group = "arm_torso"
            request.object_names = [object_name]
            
            future = self.attach_col_client.call_async(request)
            if not self._wait_future(future, 10.0):
                self.get_logger().error("❌ Attach object timeout!")
                return False
            
            response = future.result()
            if hasattr(response, 'success') and response.success:
                self.get_logger().info(f"✓ Object attached (should turn purple): {object_name}")
                return True
            else:
                self.get_logger().error(f"❌ Attach failed: {getattr(response, 'message', 'Unknown error')}")
                return False
                
        except Exception as e:
            self.get_logger().error(f"❌ Attach object failed: {e}")
            return False
    
    def detach_object(self, object_name):
        """Detach object from gripper"""
        self.get_logger().info(f"🔓 Detaching object: {object_name}")
        
        try:
            request = DettachObj.Request()
            request.move_group = "arm_torso"
            request.object_names = [object_name]
            
            future = self.detach_col_client.call_async(request)
            if not self._wait_future(future, 10.0):
                self.get_logger().error("❌ Detach object timeout!")
                return False
            
            response = future.result()
            if hasattr(response, 'success') and response.success:
                self.get_logger().info(f"✓ Object detached: {object_name}")
                return True
            else:
                self.get_logger().error(f"❌ Detach failed: {getattr(response, 'message', 'Unknown error')}")
                return False
                
        except Exception as e:
            self.get_logger().error(f"❌ Detach object failed: {e}")
            return False
    
    def delete_gazebo_entity(self, object_id):
        """
        Delete the Gazebo entity corresponding to the object
        This prevents collision checking with the physical object after it's attached to gripper
        
        Args:
            object_id: int - the object ID (1, 2, 3)
        
        Returns:
            bool - True if deletion was successful
        """
        if object_id not in self.gazebo_entity_names:
            self.get_logger().error(f"❌ Unknown object_id: {object_id}")
            return False
        
        entity_name = self.gazebo_entity_names[object_id]
        
        self.get_logger().info(f"🗑️  Deleting Gazebo entity: {entity_name} (object_{object_id})")
        
        try:
            # Wait for service if not ready
            if not self.delete_entity_client.service_is_ready():
                self.get_logger().info("   Waiting for /delete_entity service...")
                self.delete_entity_client.wait_for_service(timeout_sec=5.0)
            
            # Create and send delete request
            delete_request = DeleteEntity.Request()
            delete_request.name = entity_name
            
            future = self.delete_entity_client.call_async(delete_request)
            if not self._wait_future(future, 5.0):
                self.get_logger().error(f"❌ Delete entity timeout for {entity_name}!")
                return False
            
            response = future.result()
            if response.success:
                self.get_logger().info(f"✓ Gazebo entity deleted: {entity_name}")
                return True
            else:
                self.get_logger().warn(f"⚠️  Delete entity returned success=False: {entity_name}")
                # Some Gazebo versions don't populate success field properly, so we'll continue anyway
                return True
                
        except Exception as e:
            self.get_logger().error(f"❌ Delete Gazebo entity failed: {e}")
            return False
    
    def goto_named_target(self, target_name):
        """Move to a named target (e.g., 'home', 'carry')"""
        self.get_logger().info(f"🎯 Moving to named target: {target_name}")
        
        try:
            request = GotoNamedTarget.Request()
            request.move_group = "arm_torso"
            request.target = target_name
            
            future = self.goto_named_target_client.call_async(request)
            if not self._wait_future(future, 30.0):
                self.get_logger().error("❌ Goto named target timeout!")
                return False
            
            response = future.result()
            if hasattr(response, 'success') and response.success:
                self.get_logger().info(f"✓ Reached named target: {target_name}")
                return True
            else:
                self.get_logger().error(f"❌ Goto named target failed: {getattr(response, 'message', 'Unknown error')}")
                return False
                
        except Exception as e:
            self.get_logger().error(f"❌ Goto named target failed: {e}")
            return False
    
    def pick_object(self, object_id, grasp_x_offset=0.01, grasp_y_offset=0.0, grasp_z_offset=0.0,
                    pregrasp_offset=0.05, lift_z_offset=0.15, lift_x_offset=0.0):
        """
        Execute pick sequence for a specific object:
        1. Move to pregrasp
        2. Open gripper
        3. Move to grasp (with optional offset adjustments)
        4. Close gripper
        5. Attach object to gripper in MoveIt
        5.5. Delete Gazebo entity (prevents collision checking interference)
        6. Lift up (with Z-axis lift to clear table)
        7. Return to pregrasp position (safe carry pose)
        
        Args:
            object_id: int - ID of the object to pick (1, 2, 3)
            grasp_x_offset: float - X-axis adjustment for grasp pose (meters, default: 0.0)
            grasp_y_offset: float - Y-axis adjustment for grasp pose (meters, default: 0.0)
            grasp_z_offset: float - Z-axis adjustment for grasp pose (meters, default: 0.0)
            pregrasp_offset: float - Distance back from grasp in X (meters, default: 0.05)
            lift_z_offset: float - Height to lift after grasp (meters, default: 0.15)
            lift_x_offset: float - X offset while lifting (meters, default: 0.0)
        """
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"🎯 STARTING PICK SEQUENCE FOR OBJECT {object_id}")
        self.get_logger().info("=" * 60)
        
        # Check if we have poses for this object
        if object_id not in self.pregrasp_poses or object_id not in self.grasp_poses:
            self.get_logger().error(f"❌ Missing poses for object {object_id}")
            return False
        
        # Get the original grasp pose from scene_analysis
        original_grasp_pose = self.grasp_poses[object_id]
        object_name = f"object_{object_id}"
        
        # Apply grasp adjustments if specified
        if grasp_x_offset != 0.0 or grasp_y_offset != 0.0 or grasp_z_offset != 0.0:
            grasp_pose = self.compute_adjusted_grasp_pose(
                original_grasp_pose, 
                x_offset=grasp_x_offset,
                y_offset=grasp_y_offset,
                z_offset=grasp_z_offset
            )
            self.get_logger().info(f"📐 Grasp adjusted: X{grasp_x_offset:+.3f}, Y{grasp_y_offset:+.3f}, Z{grasp_z_offset:+.3f}")
        else:
            grasp_pose = original_grasp_pose
        
        # Compute pregrasp from adjusted grasp
        pregrasp_pose = self.compute_pregrasp_from_grasp(grasp_pose, offset=pregrasp_offset)
        
        # Compute lift pose from adjusted grasp
        lift_pose = self.compute_lift_pose(grasp_pose, z_offset=lift_z_offset, x_offset=lift_x_offset)
        
        # Step 1: Move to pregrasp position
        self.get_logger().info(f"\n[STEP 1] Moving to PREGRASP position...")
        plan_response = self.plan_to_pose(pregrasp_pose, "pregrasp")
        if not plan_response or not plan_response.success:
            self.get_logger().error("❌ Failed to plan to pregrasp")
            return False
        if not self.execute_plan(plan_response.plans):
            self.get_logger().error("❌ Failed to execute pregrasp")
            return False
        time.sleep(1.0)
        
        # Step 2: Open gripper
        self.get_logger().info(f"\n[STEP 2] Opening gripper...")
        if not self.open_gripper():
            self.get_logger().error("❌ Failed to open gripper")
            return False
        time.sleep(1.0)
        
        # Step 3: Move to grasp position
        self.get_logger().info(f"\n[STEP 3] Moving to GRASP position...")
        plan_response = self.plan_to_pose(grasp_pose, "grasp")
        if not plan_response or not plan_response.success:
            self.get_logger().error("❌ Failed to plan to grasp")
            return False
        if not self.execute_plan(plan_response.plans):
            self.get_logger().error("❌ Failed to execute grasp")
            return False
        time.sleep(1.0)
        
        # Step 4: Close gripper
        self.get_logger().info(f"\n[STEP 4] Closing gripper...")
        if not self.close_gripper():
            self.get_logger().error("❌ Failed to close gripper")
            return False
        time.sleep(1.0)
        
        # Step 5: Attach object to gripper
        self.get_logger().info(f"\n[STEP 5] Attaching object to gripper...")
        if not self.attach_object(object_name):
            self.get_logger().error("❌ Failed to attach object")
            return False
        time.sleep(1.0)
        
        # Step 5.5: Delete Gazebo entity to prevent collision checking
        self.get_logger().info(f"\n[STEP 5.5] Deleting Gazebo entity to avoid collision interference...")
        if not self.delete_gazebo_entity(object_id):
            self.get_logger().warn("⚠️  Failed to delete Gazebo entity, but continuing...")
            # Don't return False - we can continue even if deletion fails
        time.sleep(0.5)
        
        # Step 6: Lift object up (with Z-axis offset to clear table)
        self.get_logger().info(f"\n[STEP 6] Lifting object (Z+{lift_z_offset:.2f}m to clear table)...")
        plan_response = self.plan_to_pose(lift_pose, "lift")
        if not plan_response or not plan_response.success:
            self.get_logger().error("❌ Failed to plan lift")
            return False
        if not self.execute_plan(plan_response.plans):
            self.get_logger().error("❌ Failed to execute lift")
            return False
        time.sleep(1.0)
        
        # Step 7: Return to pregrasp position (safe carry pose)
        self.get_logger().info(f"\n[STEP 7] Returning to PREGRASP position (carry pose)...")
        plan_response = self.plan_to_pose(pregrasp_pose, "return to pregrasp")
        if not plan_response or not plan_response.success:
            self.get_logger().error("❌ Failed to plan return to pregrasp")
            return False
        if not self.execute_plan(plan_response.plans):
            self.get_logger().error("❌ Failed to execute return to pregrasp")
            return False
        
        self.get_logger().info("\n" + "=" * 60)
        self.get_logger().info(f"✅ PICK SEQUENCE COMPLETE FOR OBJECT {object_id}!")
        self.get_logger().info("=" * 60)
        return True
    
    def run_pick_and_carry_sequence(self, object_id=1):
        """
        Main sequence: Pick object and return to pregrasp (carry position)
        """
        self.get_logger().info("🚀 Starting pick and carry sequence...")
        self.get_logger().info(f"   Target: Object {object_id}")
        
        # Wait for markers by actively spinning
        self.get_logger().info("⏳ Waiting for grasp markers (pregrasp will be computed automatically)...")
        max_wait_time = 10.0  # Wait up to 10 seconds
        start_time = time.time()
        
        while time.time() - start_time < max_wait_time:
            # Spin to receive messages
            rclpy.spin_once(self, timeout_sec=0.1)
            
            # Check if we have the required poses (only need grasp, pregrasp is computed)
            if object_id in self.grasp_poses:
                self.get_logger().info(f"✓ Received grasp pose for object {object_id}!")
                # Verify pregrasp was computed
                if object_id in self.pregrasp_poses:
                    self.get_logger().info(f"✓ Pre-grasp pose computed for object {object_id}!")
                break
            
            # Log progress every 2 seconds
            elapsed = time.time() - start_time
            if int(elapsed) % 2 == 0 and elapsed > 0.1:
                self.get_logger().info(f"   Still waiting... ({elapsed:.1f}s elapsed)")
                self.get_logger().info(f"   Received grasp: {list(self.grasp_poses.keys())}")
                self.get_logger().info(f"   Computed pregrasp: {list(self.pregrasp_poses.keys())}")
        
        # Final check - we only need grasp pose since pregrasp is computed
        if object_id not in self.grasp_poses:
            self.get_logger().error(f"❌ Haven't received grasp pose for object {object_id} after {max_wait_time}s!")
            self.get_logger().info(f"   Available grasp poses: {list(self.grasp_poses.keys())}")
            return False
        
        if object_id not in self.pregrasp_poses:
            self.get_logger().error(f"❌ Failed to compute pregrasp pose for object {object_id}!")
            return False
        
        # Execute pick (includes lift and return to pregrasp carry position)
        if not self.pick_object(object_id):
            self.get_logger().error("❌ Pick sequence failed!")
            return False
        
        self.get_logger().info("\n" + "🎉" * 30)
        self.get_logger().info("✅ PICK AND CARRY SEQUENCE COMPLETE!")
        self.get_logger().info("🎉" * 30)
        return True


def main():
    rclpy.init()
    node = PickAndCarry()
    
    try:
        # Run the pick and carry sequence for object 1
        # You can change this to pick object 2 or 3
        success = node.run_pick_and_carry_sequence(object_id=1)
        
        if success:
            node.get_logger().info("Node will keep running. Press Ctrl+C to exit.")
            rclpy.spin(node)
        else:
            node.get_logger().error("Pick and carry sequence failed!")
            return 1
            
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user")
    except Exception as e:
        node.get_logger().error(f"Unexpected error: {e}")
        import traceback
        traceback.print_exc()
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
    
    return 0


if __name__ == "__main__":
    exit(main())