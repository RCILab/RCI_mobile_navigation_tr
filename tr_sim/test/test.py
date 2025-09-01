#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

def make_pose(x: float, y: float, yaw: float, frame: str = "map") -> PoseStamped:
    p = PoseStamped()
    p.header.frame_id = frame
    p.pose.position.x = x
    p.pose.position.y = y
    p.pose.orientation.z = math.sin(yaw * 0.5)
    p.pose.orientation.w = math.cos(yaw * 0.5)
    return p

class WaypointFollower(Node):
    def __init__(self):
        super().__init__("waypoint_follower")
        self._ac_navigate = ActionClient(self, NavigateToPose, "/navigate_to_pose")

        self.get_logger().info("Waiting for Nav2 action server ('/navigate_to_pose')...")
        self._ac_navigate.wait_for_server()
        self.get_logger().info("Nav2 action server is ready.")

    def go_to_pose(self, pose: PoseStamped, wp_idx: int) -> bool:
        self.get_logger().info(f"Sending goal WP-{wp_idx} [{pose.pose.position.x:.2f}, {pose.pose.position.y:.2f}]...")

        goal = NavigateToPose.Goal()

        pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose = pose

        future = self._ac_navigate.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error(f"Goal for WP-{wp_idx} was rejected.")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        status = result_future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"Reached WP-{wp_idx} successfully!")
            return True
        else:
            self.get_logger().error(f"Failed to reach WP-{wp_idx} (Status code: {status})")
            return False

def main():
    rclpy.init()
    node = WaypointFollower()

    waypoints = [
        make_pose(2.0, 0.0, 0.0),       # 0
        make_pose(2.0, -7.0, -1.571),   # 1
        make_pose(0.0, -7.0, -3.141),   # 2
        make_pose(2.0, -7.0, 0.0),      # 3
        make_pose(2.0, 0.0, 1.571),     # 4
        make_pose(0.0, 0.0, 3.141)      # 5
    ]

    try:
        repeat_count = int(input(">> Enter the number of round trips: "))
        if repeat_count <= 0: raise ValueError
    except ValueError:
        node.get_logger().error("Invalid input. Please enter an integer greater than 0.")
        rclpy.shutdown()
        return

    try:
        for i in range(repeat_count):
            node.get_logger().info(f"\n★★★★★ Starting round trip {i+1}/{repeat_count} ★★★★★")
            success = True
            for wp_idx, pose in enumerate(waypoints):
                success = node.go_to_pose(pose, wp_idx)
                if not success:
                    node.get_logger().error(f"Failed to navigate to WP-{wp_idx}! Aborting current trip.")
                    break

            if not success:
                break

            node.get_logger().info(f"Round trip {i+1} completed!")

    except KeyboardInterrupt:
        node.get_logger().info("\nExecution interrupted by user.")
    except Exception as e:
        node.get_logger().error(f"An exception occurred during execution: {e}")
    finally:
        node.get_logger().info("Shutting down the script.")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()