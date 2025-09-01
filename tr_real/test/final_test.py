#!/usr/bin/env python3
import math
import time
import rclpy
import numpy as np
import csv
import matplotlib.pyplot as plt
from datetime import datetime
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from tr_driver_msgs.msg import PgvData

def make_pose(x: float, y: float, yaw: float, frame: str = "map") -> PoseStamped:
    p = PoseStamped()
    p.header.frame_id = frame
    p.header.stamp = rclpy.clock.Clock().now().to_msg()
    p.pose.position.x = x
    p.pose.position.y = y
    p.pose.orientation.z = math.sin(yaw * 0.5)
    p.pose.orientation.w = math.cos(yaw * 0.5)
    return p

class AgvPerformanceTester(Node):
    def __init__(self):
        super().__init__("agv_performance_tester")
        self._ac_navigate = ActionClient(self, NavigateToPose, "/navigate_to_pose")

        self.pgv_sub = self.create_subscription(PgvData, '/pgv', self.pgv_callback, 10)
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.WP_TO_TAG_MAP = {
            0: 16,
            1: 17,
            2: 18,
            3: 17,
            4: 16,
            5: 15
        }
        self.STOP_TOLERANCE_MM = 10.0
        self.PATH_TARGET_TAG_IDS = {19}
        self.PATH_Y_TOLERANCE_MM = 30.0
        self.is_monitoring_path = False
        self.last_pgv_msg = None

        self.stop_position_data = []
        self.path_y_position_data = []
        self.current_path_readings = {}

        self._ac_navigate.wait_for_server()
        self.get_logger().info("Action server is ready.")

    def pgv_callback(self, msg: PgvData):
        self.last_pgv_msg = msg
        if not msg.tag_detected or not self.is_monitoring_path:
            return

        if msg.tag_id in self.PATH_TARGET_TAG_IDS:
            if msg.tag_id not in self.current_path_readings:
                self.current_path_readings[msg.tag_id] = []
            self.current_path_readings[msg.tag_id].append(msg.y_pos)

            y_abs_mm = abs(msg.y_pos)
            is_within_tolerance = y_abs_mm <= self.PATH_Y_TOLERANCE_MM
            if not is_within_tolerance:
                self.get_logger().warn(f"Path deviation detected! Tag ID: {msg.tag_id}, Y-Offset: {msg.y_pos:.2f} mm")

    def perform_origin_alignment(self) -> bool:
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("Starting origin alignment. Please place the robot over Tag ID 15.")
        self.get_logger().info(f"Target: Within {self.STOP_TOLERANCE_MM:.1f} mm from the tag center.")
        self.get_logger().info("When aligned, enter 'y' to set the origin. (Abort: Ctrl+C)")
        self.get_logger().info("="*60)
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.last_pgv_msg is None:
                self.get_logger().warn("Waiting for PGV data...", throttle_duration_sec=2)
                continue
            if not self.last_pgv_msg.tag_detected or self.last_pgv_msg.tag_id != self.WP_TO_TAG_MAP[5]:
                print(f"\r\033[KSearching for Tag 15... (Detected: {self.last_pgv_msg.tag_detected}, ID: {self.last_pgv_msg.tag_id})", end="")
                continue
            x_mm, y_mm = self.last_pgv_msg.x_pos, self.last_pgv_msg.y_pos
            distance_mm = math.sqrt(x_mm**2 + y_mm**2)
            if distance_mm <= self.STOP_TOLERANCE_MM:
                print(f"\r\033[KAligned! X: {x_mm:6.2f} mm, Y: {y_mm:6.2f} mm, Dist: {distance_mm:6.2f} mm. Set this as origin? (y/n): ", end="")
                try:
                    answer = input()
                    if answer.lower() == 'y':
                        self.get_logger().info("\nOrigin set.")
                        return True
                except (EOFError, KeyboardInterrupt): return False
            else:
                print(f"\r\033[KAligning... X: {x_mm:6.2f} mm, Y: {y_mm:6.2f} mm, Dist: {distance_mm:6.2f} mm", end="")
        return False

    def measure_stopping_accuracy(self, wp_idx: int):
        if wp_idx not in self.WP_TO_TAG_MAP:
            return
        target_tag_id = self.WP_TO_TAG_MAP[wp_idx]
        self.get_logger().info(f"--- Starting final position measurement for WP-{wp_idx} (Tag ID: {target_tag_id}) ---")

        time.sleep(0.8)
        self.get_logger().info("  - Waiting to receive latest data.")

        start_time = self.get_clock().now()
        timeout_sec = 2.0
        valid_msg_received = False

        while (self.get_clock().now() - start_time).nanoseconds / 1e9 < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.05)
            if (self.last_pgv_msg and
                self.last_pgv_msg.tag_detected and
                self.last_pgv_msg.tag_id == target_tag_id):
                self.get_logger().info(f"  - Received valid PGV data (Tag ID: {target_tag_id})")
                valid_msg_received = True
                break
            time.sleep(0.1)

        if not valid_msg_received:
            self.get_logger().error(f"Failed to find expected Tag ID ({target_tag_id}) at WP-{wp_idx} within timeout.")
            if self.last_pgv_msg:
                 self.get_logger().error(f"  - Last detected tag: ID={self.last_pgv_msg.tag_id}, Detected={self.last_pgv_msg.tag_detected}")
            return

        x_mm, y_mm = self.last_pgv_msg.x_pos, self.last_pgv_msg.y_pos
        distance_error_mm = math.sqrt(x_mm**2 + y_mm**2)
        is_within_tolerance = distance_error_mm <= self.STOP_TOLERANCE_MM
        self.stop_position_data.append({
            "timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f"),
            "wp": wp_idx,
            "tag_id": target_tag_id,
            "x_pos_mm": x_mm,
            "y_pos_mm": y_mm
        })
        result_str = "PASS" if is_within_tolerance else "FAIL"
        self.get_logger().info(f"  - Tag center error: {distance_error_mm:.3f} mm -> [{result_str}]")

    def go_to_pose(self, pose: PoseStamped, wp_idx: int) -> bool:
        self.get_logger().info(f"Sending goal WP-{wp_idx} [{pose.pose.position.x:.3f}, {pose.pose.position.y:.3f}]...")
        goal = NavigateToPose.Goal()
        goal.pose = pose

        self.current_path_readings.clear()
        self.is_monitoring_path = True
        future = self._ac_navigate.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal was rejected.")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        status = result_future.result().status
        self.is_monitoring_path = False
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"Reached WP-{wp_idx} successfully!")

            for tag_id, y_readings in self.current_path_readings.items():
                if y_readings:
                    avg_y_pos = np.mean(y_readings)
                    self.path_y_position_data.append({
                        "timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f"),
                        "tag_id": tag_id,
                        "y_pos_mm": avg_y_pos
                    })

            if wp_idx in self.WP_TO_TAG_MAP:
                self.get_logger().info(f"  - WP-{wp_idx} is a measurement point. Measuring stopping accuracy.")
                self.measure_stopping_accuracy(wp_idx)
            return True
        else:
            self.get_logger().error(f"Failed to reach WP-{wp_idx} (Status: {status})")
            return False

    def reset_localization_to_origin(self):
        self.get_logger().info("Resetting initial pose to (0, 0)...")
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.get_clock().now().to_msg()
        initial_pose.pose.pose.position.x = 0.0
        initial_pose.pose.pose.position.y = 0.0
        initial_pose.pose.pose.orientation.w = 1.0
        self.initial_pose_pub.publish(initial_pose)
        self.get_logger().info("Published initial pose to `/initialpose` topic.")
        time.sleep(1.0)

    def save_and_plot_results(self):
        self.get_logger().info("\n" + "="*60 + "\nSaving data and plotting results... \n" + "="*60)
        if self.stop_position_data:
            with open('stop_positions.csv', 'w', newline='') as f:
                writer = csv.DictWriter(f, fieldnames=self.stop_position_data[0].keys())
                writer.writeheader()
                writer.writerows(self.stop_position_data)
            self.get_logger().info("'stop_positions.csv' saved successfully.")

            plt.figure(figsize=(10, 10))

            plot_styles = {
                0: {'marker': 'o', 'color': '#ff7f0e', 'label': f'WP0 (Tag 16)'},
                1: {'marker': 's', 'color': '#2ca02c', 'label': f'WP1 (Tag 17)'},
                2: {'marker': 'D', 'color': '#d62728', 'label': f'WP2 (Tag 18)'},
                3: {'marker': 's', 'color': '#9467bd', 'label': f'WP3 (Tag 17)'},
                4: {'marker': 'o', 'color': '#8c564b', 'label': f'WP4 (Tag 16)'},
                5: {'marker': '^', 'color': '#1f77b4', 'label': f'WP5 (Tag 15)'}
            }

            waypoints_in_data = sorted(list(set(d['wp'] for d in self.stop_position_data)))

            for wp_idx in waypoints_in_data:
                wp_data = [d for d in self.stop_position_data if d['wp'] == wp_idx]
                if wp_data:
                    style = plot_styles.get(wp_idx, {'marker': 'x', 'color': 'black', 'label': f'WP{wp_idx}'})
                    plt.scatter(
                        [d['x_pos_mm'] for d in wp_data],
                        [d['y_pos_mm'] for d in wp_data],
                        marker=style['marker'],
                        color=style['color'],
                        label=style['label'],
                        alpha=0.7,
                        edgecolors='k',
                        s=80
                    )

            plt.title('Stop Position Accuracy at Waypoints')
            plt.xlabel('X Position (mm)')
            plt.ylabel('Y Position (mm)')
            plt.axhline(0, color='grey', lw=0.5)
            plt.axvline(0, color='grey', lw=0.5)
            plt.grid(True, linestyle='--', alpha=0.6)
            plt.gca().set_aspect('equal', adjustable='box')
            all_x = [d['x_pos_mm'] for d in self.stop_position_data]
            all_y = [d['y_pos_mm'] for d in self.stop_position_data]
            max_val = self.STOP_TOLERANCE_MM
            if all_x and all_y:
                max_val = max(max(map(abs, all_x)), max(map(abs, all_y)), self.STOP_TOLERANCE_MM)
            plt.xlim(-max_val * 1.2, max_val * 1.2)
            plt.ylim(-max_val * 1.2, max_val * 1.2)
            circle = plt.Circle((0, 0), self.STOP_TOLERANCE_MM, color='r', fill=False, linestyle='--', label=f'Tolerance ({self.STOP_TOLERANCE_MM}mm)')
            plt.gca().add_artist(circle)
            plt.legend()
            plt.savefig('stop_positions_scatter.png')
            plt.close()
            self.get_logger().info("'stop_positions_scatter.png' saved successfully.")

        if self.path_y_position_data:
            with open('path_y_positions.csv', 'w', newline='') as f:
                writer = csv.DictWriter(f, fieldnames=self.path_y_position_data[0].keys())
                writer.writeheader()
                writer.writerows(self.path_y_position_data)
            self.get_logger().info("'path_y_positions.csv' saved successfully.")

            y_vals = [d['y_pos_mm'] for d in self.path_y_position_data]
            plt.figure(figsize=(12, 8))
            plt.scatter(range(len(y_vals)), y_vals, alpha=0.7, edgecolors='k')
            plt.title('Path Following Accuracy (Y-position at each Tag)')
            plt.xlabel('Measurement Sequence')
            plt.ylabel('Average Y Position (mm)')
            plt.axhline(0, color='black', lw=1)
            plt.axhline(self.PATH_Y_TOLERANCE_MM, color='r', linestyle='--', label=f'Tolerance (+{self.PATH_Y_TOLERANCE_MM}mm)')
            plt.axhline(-self.PATH_Y_TOLERANCE_MM, color='r', linestyle='--', label=f'Tolerance (-{self.PATH_Y_TOLERANCE_MM}mm)')
            plt.grid(True, linestyle='--', alpha=0.6, axis='y')
            plt.legend()
            plt.savefig('path_y_positions_scatter.png')
            plt.close()
            self.get_logger().info("'path_y_positions_scatter.png' saved successfully.")

def main():
    rclpy.init()
    node = AgvPerformanceTester()

    waypoints = [
        make_pose(2.0+0.04882, 0.0-0.05571+0.03884, 0.0),
        make_pose(2.0+0.04882, -7.0-0.04464+0.03588, -1.571),
        make_pose(0.0-0.03495, -7.0-0.04464, -3.141),
        make_pose(2.0-0.0069, -7.0-0.04464+0.07255, 0.0),
        make_pose(2.0-0.0069, 0.0-0.05571-0.013, 1.571),
        make_pose(0.0+0.01235, 0.0-0.05571, 3.141)
    ]

    lap_times = []

    try:
        repeat_count = int(input(">> Enter the number of test laps: "))
        if repeat_count <= 0: raise ValueError
    except ValueError:
        node.get_logger().error("Invalid input. Please enter an integer greater than 0.")
        rclpy.shutdown()
        return

    try:
        if not node.perform_origin_alignment():
            raise KeyboardInterrupt
        node.reset_localization_to_origin()

        for i in range(repeat_count):
            node.get_logger().info(f"\n★★★★★ Starting Lap {i+1}/{repeat_count} ★★★★★")
            lap_start_time = time.time()
            success = True

            for wp_idx, pose in enumerate(waypoints):
                success = node.go_to_pose(pose, wp_idx)
                if not success:
                    node.get_logger().error(f"Failed to navigate to WP-{wp_idx}! Aborting current lap.")
                    break

            if not success:
                break

            lap_end_time = time.time()
            lap_duration = lap_end_time - lap_start_time
            lap_times.append(lap_duration)
            node.get_logger().info(f"Lap {i+1} completed! (Duration: {lap_duration:.2f} seconds)")

    except KeyboardInterrupt:
        node.get_logger().info("\nTest interrupted by user.")
    except Exception as e:
        node.get_logger().error(f"An exception occurred during the test: {e}")
    finally:
        if lap_times:
            avg_lap_time = np.mean(lap_times)
            node.get_logger().info("\n" + "="*60)
            node.get_logger().info(f"Final Result: Average lap time: {avg_lap_time:.2f} seconds")
            node.get_logger().info("="*60)

        node.save_and_plot_results()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()