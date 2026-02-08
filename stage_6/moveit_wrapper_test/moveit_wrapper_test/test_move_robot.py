import sys
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from moveit_wrapper_interfaces.msg import Waypoint
from moveit_wrapper_interfaces.srv import Plan, ExecutePlans
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class TestMoveRobot(Node):
    def __init__(self):
        super().__init__("test_move_robot")
        self.pub = self.create_publisher(Marker, 'move_markers', 10)
        self.plan_client = self.create_client(Plan, '/moveit_wrapper/plan')
        self.execute_client = self.create_client(ExecutePlans, '/moveit_wrapper/execute_plans')
        self.head_action_client = ActionClient(
            self, FollowJointTrajectory, '/head_controller/follow_joint_trajectory'
        )
        self.wait_for_services()

    def wait_for_services(self):
        while not self.plan_client.wait_for_service(timeout_sec=1.0):
            pass
        while not self.execute_client.wait_for_service(timeout_sec=1.0):
            pass
        self.head_action_client.wait_for_server()

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

    def create_pose1(self):
        pose1 = PoseStamped()
        pose1.header.frame_id = "base_link"
        pose1.header.stamp = self.get_clock().now().to_msg()
        pose1.pose.position.x = 0.5
        pose1.pose.position.y = 0.5
        pose1.pose.position.z = 0.8
        pose1.pose.orientation.x = 0.0
        pose1.pose.orientation.y = 0.0
        pose1.pose.orientation.z = 0.0
        pose1.pose.orientation.w = 1.0
        return pose1

    def create_move_marker1(self, pose1):
        move_marker1 = Marker()
        move_marker1.header.frame_id = pose1.header.frame_id
        move_marker1.header.stamp = self.get_clock().now().to_msg()
        move_marker1.ns = "move_target"
        move_marker1.id = 0
        move_marker1.type = Marker.ARROW
        move_marker1.action = Marker.ADD
        move_marker1.pose = pose1.pose
        move_marker1.scale.x = 0.2
        move_marker1.scale.y = 0.05
        move_marker1.scale.z = 0.05
        move_marker1.color.r = 1.0
        move_marker1.color.g = 0.0
        move_marker1.color.b = 0.0
        move_marker1.color.a = 1.0
        move_marker1.lifetime.sec = 10
        return move_marker1

    def publish_move_marker(self, marker):
        self.pub.publish(marker)

    def create_waypoint(self, pose1):
        wp1 = Waypoint()
        wp1.pose = pose1
        return wp1

    def call_plan_service(self, waypoint):
        request = Plan.Request()
        request.waypoints = [waypoint]
        request.move_group = "arm_torso"
        future = self.plan_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
        response = future.result()
        return response if response and response.success else None

    def call_execute_service(self, plans):
        request = ExecutePlans.Request()
        request.plans = plans
        request.move_group = "arm_torso"
        future = self.execute_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=60.0)
        response = future.result()
        return response if response and response.success else None

    def run_test(self):
        if not self.move_head_to_look_down(pan=0.0, tilt=-0.9):
            pass
        time.sleep(2.0)
        
        pose1 = self.create_pose1()
        move_marker1 = self.create_move_marker1(pose1)
        self.publish_move_marker(move_marker1)
        time.sleep(3.0)
        
        wp1 = self.create_waypoint(pose1)
        plan_response = self.call_plan_service(wp1)
        if plan_response is None:
            return False
        
        execute_response = self.call_execute_service(plan_response.plans)
        return execute_response is not None

def main():
    rclpy.init()
    node = TestMoveRobot()
    try:
        success = node.run_test()
        if success:
            rclpy.spin(node)
            sys.exit(0)
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