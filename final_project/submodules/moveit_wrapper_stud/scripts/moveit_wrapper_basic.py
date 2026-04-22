#!/usr/bin/env python3

import sys
import time
import random

import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point, PoseStamped
from moveit_msgs.msg import Constraints, RobotState
from moveit_wrapper_interfaces.msg import Waypoint
from moveit_wrapper_interfaces.srv import (AddColObj, AttachObj, DettachObj,
                                           ExecutePlans, GetAttachedObjs,
                                           GetColObj, GotoNamedTarget, Plan,
                                           RemoveColObj)

from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker

# Custom interfaces


class MoveitWrapperTest(Node):
    def __init__(self):
        super().__init__("grasp_pipeline_test")

        self.plan_client = self.create_client(Plan, "/moveit_wrapper/plan")
        self.execute_client = self.create_client(
            ExecutePlans, "/moveit_wrapper/execute_plans"
        )
        self.goto_named_target_client = self.create_client(
            GotoNamedTarget, "/moveit_wrapper/goto_named_target"
        )

        # Planning-scene & attach/detach
        self.add_col_client = self.create_client(
            AddColObj, "/moveit_wrapper/add_collision_object"
        )
        self.remove_col_client = self.create_client(
            RemoveColObj, "/moveit_wrapper/remove_collision_object"
        )
        self.get_col_client = self.create_client(
            GetColObj, "/moveit_wrapper/get_collision_objects"
        )
        self.attach_col_client = self.create_client(
            AttachObj, "/moveit_wrapper/attach_object"
        )
        self.detach_col_client = self.create_client(
            DettachObj, "/moveit_wrapper/detach_object"
        )
        self.get_attached_client = self.create_client(
            GetAttachedObjs, "/moveit_wrapper/get_attached_objects"
        )

        self.wait_for_services()

    # -------------------------------
    # Service waiting
    # -------------------------------
    def wait_for_services(self):
        """Wait for all required services to be available."""
        self.get_logger().info("Waiting for services...")

        services = [
            (self.plan_client, "/moveit_wrapper/plan"),
            (self.execute_client, "/moveit_wrapper/execute_plans"),
            (self.goto_named_target_client, "/moveit_wrapper/goto_named_target"),
            (self.add_col_client, "/moveit_wrapper/add_collision_object"),
            (self.remove_col_client, "/moveit_wrapper/remove_collision_object"),
            (self.get_col_client, "/moveit_wrapper/get_collision_objects"),
            (self.attach_col_client, "/moveit_wrapper/attach_object"),
            (self.detach_col_client, "/moveit_wrapper/detach_object"),
            (self.get_attached_client, "/moveit_wrapper/get_attached_objects"),
        ]

        for client, service_name in services:
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(
                    f"Service {service_name} not available, waiting..."
                )

        self.get_logger().info("All services are available!")

    # -------------------------------
    # Utility helpers
    # -------------------------------
    def _wait_future(self, future, timeout_sec: float) -> bool:
        start = time.time()
        while rclpy.ok() and not future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start > timeout_sec:
                return False
        return future.done()

    # -------------------------------
    # Planning service wrappers
    # -------------------------------
    def call_planning_service(self, waypoints, description=""):
        """Call the motion planning service."""
        if description:
            self.get_logger().info(
                f"Calling motion planning service for {description}..."
            )
        else:
            self.get_logger().info("Calling motion planning service...")

        request = Plan.Request()
        request.waypoints = waypoints
        request.path_constraints = Constraints()
        request.send_partial = True
        request.use_start_state = False
        request.start_state = RobotState()
        request.move_group = "arm_torso"

        self.get_logger().info(f"Planning for {len(waypoints)} waypoints:")
        for i, wp in enumerate(waypoints):
            self.get_logger().info(
                f"  Waypoint {i+1} frame: {wp.pose.header.frame_id} "
                f"pos: [{wp.pose.pose.position.x:.3f}, {wp.pose.pose.position.y:.3f}, {wp.pose.pose.position.z:.3f}]"
            )

        try:
            future = self.plan_client.call_async(request)
            if not self._wait_future(future, 15.0):
                self.get_logger().error("Planning service call timed out!")
                return None

            response = future.result()
            if response.success:
                self.get_logger().info(
                    f"Planning successful! Generated {len(response.plans)} trajectory segments"
                )
                self.get_logger().info(f"Message: {response.message}")
                return response
            else:
                self.get_logger().error(f"Planning failed: {response.message}")
                return None

        except Exception as e:
            self.get_logger().error(f"Service call failed: {str(e)}")
            return None

    def call_execution_service(self, plans):
        """Call the trajectory execution service."""
        self.get_logger().info("Calling trajectory execution service...")

        request = ExecutePlans.Request()
        request.plans = plans

        request.move_group = "arm_torso"
        try:
            future = self.execute_client.call_async(request)
            if not self._wait_future(future, 30.0):
                self.get_logger().error("Execution service call timed out!")
                return None

            response = future.result()
            if response.success:
                self.get_logger().info("Trajectory execution successful!")
                self.get_logger().info(f"Message: {response.message}")
                return response
            else:
                self.get_logger().error(
                    f"Trajectory execution failed: {response.message}"
                )
                return None

        except Exception as e:
            self.get_logger().error(f"Service call failed: {str(e)}")
            return None

    # -------------------------------
    # Planning scene helpers (all return bool)
    # -------------------------------
    def clear_planning_scene(self) -> bool:
        """
        Detach all attached objects, then remove all world objects (empty list = all).
        Returns True on success; False on any failure.
        """
        self.get_logger().info("Clearing planning scene (detach all, remove all)...")

        # Get attached
        try:
            get_req = GetAttachedObjs.Request()
            get_future = self.get_attached_client.call_async(get_req)
            if not self._wait_future(get_future, 10.0):
                self.get_logger().error("GetAttachedObjs timeout.")
                return False

            attached = list(getattr(get_future.result(), "objs", []))
        except Exception as e:
            self.get_logger().error(f"GetAttachedObjs failed: {e}")
            return False

        # Detach if any
        try:
            names_to_detach = [m.ns if m.ns else f"{m.ns}_{m.id}" for m in attached]
            if names_to_detach:
                self.get_logger().info(f"Detaching attached objects: {names_to_detach}")
                dreq = DettachObj.Request()
                dreq.object_names = names_to_detach
                dfut = self.detach_col_client.call_async(dreq)
                if not self._wait_future(dfut, 15.0):
                    self.get_logger().error("DettachObj timeout.")
                    return False
                dresp = dfut.result()
                if hasattr(dresp, "success") and dresp.success is False:
                    self.get_logger().error(
                        f"DettachObj failed: {getattr(dresp, 'message', '')}"
                    )
                    return False
        except Exception as e:
            self.get_logger().error(f"DettachObj failed: {e}")
            return False

        # Remove all world objects
        try:
            rreq = RemoveColObj.Request()
            rreq.object_names = []  # empty = remove all
            r_fut = self.remove_col_client.call_async(rreq)
            if not self._wait_future(r_fut, 15.0):
                self.get_logger().error("RemoveColObj timeout.")
                return False
            rresp = r_fut.result()
            if hasattr(rresp, "success") and rresp.success is False:
                self.get_logger().error(
                    f"RemoveColObj failed: {getattr(rresp, 'message', '')}"
                )
                return False
            self.get_logger().info("World objects removed.")
        except Exception as e:
            self.get_logger().error(f"RemoveColObj failed: {e}")
            return False

        return True

    def add_coll_objects_from_markers(self, markers):
        """Transform given markers to collision objects. Returns True on success."""

        try:
            req = AddColObj.Request()
            req.objects = markers
            fut = self.add_col_client.call_async(req)
            if not self._wait_future(fut, 15.0):
                self.get_logger().error("AddColObj timeout.")
                return False, []

            resp = fut.result()
            if hasattr(resp, "success") and resp.success is False:
                self.get_logger().error(
                    f"AddColObj failed: {getattr(resp, 'message', '')}"
                )
                return False, []

            self.get_logger().info(
                f"Added {len(markers)} collision markers to the planning scene."
            )
        except Exception as e:
            self.get_logger().error(f"AddColObj failed: {e}")
            return False, []

    def attach_objects(self, names) -> bool:
        """Attach by object names. Returns True on success."""
        if not names:
            self.get_logger().warn("attach_objects called with empty list.")
            return True
        try:
            req = AttachObj.Request()
            req.move_group = "arm_torso"
            req.object_names = names
            fut = self.attach_col_client.call_async(req)
            if not self._wait_future(fut, 15.0):
                self.get_logger().error("AttachObj timeout.")
                return False
            resp = fut.result()
            if hasattr(resp, "success") and resp.success is False:
                self.get_logger().error(
                    f"AttachObj failed: {getattr(resp, 'message', '')}"
                )
                return False
            self.get_logger().info(f"Attached objects: {names}")
            return True
        except Exception as e:
            self.get_logger().error(f"AttachObj failed: {e}")
            return False

    def detach_objects(self, names) -> bool:
        """Detach by object names. Returns True on success."""
        if not names:
            self.get_logger().warn("detach_objects called with empty list.")
            return True
        try:
            req = DettachObj.Request()
            req.move_group = "arm_torso"
            req.object_names = names
            fut = self.detach_col_client.call_async(req)
            if not self._wait_future(fut, 15.0):
                self.get_logger().error("DettachObj timeout.")
                return False
            resp = fut.result()
            if hasattr(resp, "success") and resp.success is False:
                self.get_logger().error(
                    f"DettachObj failed: {getattr(resp, 'message', '')}"
                )
                return False
            self.get_logger().info(f"Detached objects: {names}")
            return True
        except Exception as e:
            self.get_logger().error(f"DettachObj failed: {e}")
            return False

    # -------------------------------
    # Test
    # -------------------------------
    def run_moveit_wrapper_test(self):

        self.get_logger().info("Starting moveit wrapper test...")

        # Clear planning scene first (abort if fails)
        if not self.clear_planning_scene():
            self.get_logger().error("Failed to clear planning scene. Aborting.")
            return False

        marker = Marker()
        marker.header.frame_id = "base_link"  # The reference frame
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = "test"
        marker.type = Marker.CUBE  # Marker type
        marker.action = Marker.ADD  # Add/modify the marker

        # Marker position and orientation
        marker.pose.position.x = 0.99
        marker.pose.position.y = 0.06
        marker.pose.position.z = 0.369
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        # Marker dimensions
        marker.scale.x = 0.5  # width
        marker.scale.y = 0.5  # length
        marker.scale.z = 0.1  # height

        markers = [marker]

        self.add_coll_objects_from_markers(markers)

        wp1 = Waypoint()

        pose1 = PoseStamped()
        pose1.header.frame_id = "base_link"
        pose1.pose.position.x = 0.5
        pose1.pose.position.y = 0.5
        pose1.pose.position.z = 0.8
        pose1.pose.orientation.x = 0.0
        pose1.pose.orientation.y = 0.0
        pose1.pose.orientation.z = 0.0
        pose1.pose.orientation.w = 1.0


        wp1.pose = pose1

        waypoints = [wp1]

        planning_response = self.call_planning_service(waypoints)
        if not planning_response or not planning_response.success:
            self.get_logger().error(
                f"Planning failed, skipping execution."
            )

        execution_response = self.call_execution_service(
            planning_response.plans
        )
        
        return True

def main():
    rclpy.init()
    node = MoveitWrapperTest()

    try:
        success = node.run_moveit_wrapper_test()
        if success:
            node.get_logger().info("Test completed successfully!")
        else:
            node.get_logger().error("Test failed!")
            sys.exit(1)

    except KeyboardInterrupt:
        node.get_logger().info("Test interrupted by user")
    except Exception as e:
        node.get_logger().error(f"Unexpected error: {str(e)}")
        sys.exit(1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
