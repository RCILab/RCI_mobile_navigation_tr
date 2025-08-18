#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints
def make_pose(x: float, y: float, yaw: float, frame: str = "map") -> PoseStamped:
    p = PoseStamped()
    p.header.frame_id = frame
    p.pose.position.x = x
    p.pose.position.y = y
    p.pose.position.z = 0.0
    p.pose.orientation.z = math.sin(yaw * 0.5)
    p.pose.orientation.w = math.cos(yaw * 0.5)
    return p
class FollowWaypointsClient(Node):
    def __init__(self):
        super().__init__("follow_waypoints_client")
        self._ac = ActionClient(self, FollowWaypoints, "/follow_waypoints")
    def send_goal(self, poses):
        goal = FollowWaypoints.Goal()
        goal.poses = poses
        self.get_logger().info("Waiting for /follow_waypoints action server...")
        if not self._ac.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Action server not available!")
            return None
        self.get_logger().info(f"Sending {len(poses)} waypoints")
        send_future = self._ac.send_goal_async(goal, feedback_callback=self._feedback_cb)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error("Goal rejected")
            return None
        self.get_logger().info("Goal accepted. Navigating...")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result_msg = result_future.result()
        self.get_logger().info(f"Finished with status: {result_msg.status}")
        result = result_msg.result
        self.get_logger().info(f"missed_waypoints: {list(result.missed_waypoints)}")
        return result
    def _feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(f"[FB] current_waypoint: {fb.current_waypoint}")
def main():
    rclpy.init()
    node = FollowWaypointsClient()
    # 여기에 원하는 좌표 찍기 map 좌표계 기준!
    poses = [
        make_pose( 2.0,  0.0, 0.0),
        make_pose( 2.0,  0.0, math.pi),
        make_pose(0.0, 0.0,  math.pi),
        make_pose(0.0, 0.0,  0.0),
        # make_pose( 3.0,  -0.5,  0.0),
        # make_pose( 3.0,  0.0,  0.0),
    ]
    now = node.get_clock().now().to_msg()
    for p in poses:
        p.header.stamp = now
    node.send_goal(poses)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == "__main__":
    main()