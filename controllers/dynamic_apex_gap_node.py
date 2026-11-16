#!/usr/bin/env python3
"""
ROS 2 Dynamic Apex & Kinematic Braking Reactive Controller Node
Designed for F1Tenth Autonomous Racing - Sao Paulo (Interlagos) Track
Group 5 Competition - Tongmyong University

Features:
- UNC Chapel Hill Disparity Extender with Dynamic Safety Expansion
- Obstacle Safety Bubble Inflation
- Anti-Zigzag Center/Deepest Point Gap Blending
- Lateral Tail-Strike & Barrier Protection Guard
- Kinematic Threshold Lookahead Braking Curve
- Real-Time Lap Counter & Microsecond Lap Timer
- RViz 3D Text Marker HUD Publisher (/lap_hud_marker)
- CSV Telemetry Logging
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
import numpy as np
import time
import os
import csv


class DynamicApexGapNode(Node):
    def __init__(self):
        super().__init__('dynamic_apex_gap_node')

        # Parameter Declarations
        self.declare_parameter('max_speed', 8.8)
        self.declare_parameter('min_speed', 2.35)
        self.declare_parameter('a_brake', 5.8)
        self.declare_parameter('d_margin', 1.85)
        self.declare_parameter('fov_degrees', 135.0)
        self.declare_parameter('bubble_radius', 0.45)
        self.declare_parameter('car_width_half', 0.31)
        self.declare_parameter('disparity_threshold', 0.38)
        self.declare_parameter('gap_distance_threshold', 1.15)
        self.declare_parameter('alpha_blend', 0.65)
        self.declare_parameter('steer_rate_limit', 0.15)
        self.declare_parameter('csv_log_path', 'champion_lap_times.csv')

        # Fetch Parameters
        self.max_speed = self.get_parameter('max_speed').value
        self.min_speed = self.get_parameter('min_speed').value
        self.a_brake = self.get_parameter('a_brake').value
        self.d_margin = self.get_parameter('d_margin').value
        self.fov_rad = np.radians(self.get_parameter('fov_degrees').value)
        self.bubble_radius = self.get_parameter('bubble_radius').value
        self.car_width_half = self.get_parameter('car_width_half').value
        self.disparity_threshold = self.get_parameter('disparity_threshold').value
        self.gap_dist_thresh = self.get_parameter('gap_distance_threshold').value
        self.alpha = self.get_parameter('alpha_blend').value
        self.steer_max_rate = self.get_parameter('steer_rate_limit').value
        self.csv_path = self.get_parameter('csv_log_path').value

        self.prev_steer = 0.0

        # ROS 2 Publishers & Subscribers
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.marker_pub = self.create_publisher(Marker, '/lap_hud_marker', 10)

        # Lap Counter & Timer State
        self.start_pose = np.array([0.0, 0.0])
        self.lap_count = 0
        self.lap_times = []
        self.lap_start_time = None
        self.has_left_start = False
        self.total_laps_target = 10
        self.finish_gate_radius = 2.2
        self.arm_distance_threshold = 15.0

        # Initialize CSV logging
        if not os.path.exists(self.csv_path):
            with open(self.csv_path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['Lap', 'Lap_Time_Seconds', 'Best_Lap_Time_Seconds', 'Timestamp'])

        self.get_logger().info(
            f"🏆 DynamicApexGapNode Active! Target: Sao Paulo (Group 5) | Max Speed: {self.max_speed} m/s"
        )

    def odom_callback(self, msg: Odometry):
        curr_x = msg.pose.pose.position.x
        curr_y = msg.pose.pose.position.y
        curr_pos = np.array([curr_x, curr_y])
        curr_time = self.get_clock().now().nanoseconds / 1e9

        if self.lap_start_time is None:
            self.lap_start_time = curr_time

        dist_to_start = np.linalg.norm(curr_pos - self.start_pose)

        # Arm the gate when vehicle is far enough from start line
        if dist_to_start > self.arm_distance_threshold:
            self.has_left_start = True

        # Trigger lap completion when vehicle crosses the finish line
        if self.has_left_start and dist_to_start < self.finish_gate_radius:
            lap_duration = curr_time - self.lap_start_time
            if lap_duration > 10.0:  # debounce filter
                self.lap_count += 1
                self.lap_times.append(lap_duration)
                self.lap_start_time = curr_time
                self.has_left_start = False
                best_lap = min(self.lap_times)

                self.get_logger().info(
                    f"🏁 LAP {self.lap_count}/{self.total_laps_target} COMPLETED! "
                    f"Time: {lap_duration:.3f} s | Fastest: {best_lap:.3f} s"
                )

                # Append to CSV
                with open(self.csv_path, 'a', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerow([self.lap_count, f"{lap_duration:.4f}", f"{best_lap:.4f}", time.strftime('%Y-%m-%d %H:%M:%S')])

        # Publish 3D HUD marker for RViz
        self.publish_hud(curr_time, curr_x, curr_y)

    def publish_hud(self, curr_time, x, y):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "lap_hud"
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 1.2
        marker.scale.z = 0.55

        elapsed = curr_time - self.lap_start_time if self.lap_start_time else 0.0
        best_str = f"{min(self.lap_times):.2f}s" if self.lap_times else "--.--"

        marker.text = (
            f"SAO PAULO GRAND PRIX\n"
            f"LAP: {self.lap_count}/{self.total_laps_target}\n"
            f"CURRENT: {elapsed:05.2f}s\n"
            f"FASTEST: {best_str}"
        )

        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.3
        marker.color.a = 1.0
        self.marker_pub.publish(marker)

    def scan_callback(self, scan_msg: LaserScan):
        ranges = np.array(scan_msg.ranges)
        num_beams = len(ranges)
        scan_angles = np.linspace(scan_msg.angle_min, scan_msg.angle_max, num_beams)
        angle_inc = scan_msg.angle_increment

        # 1. FOV Windowing (Focus on forward driving cone)
        fov_mask = (scan_angles >= -self.fov_rad / 2.0) & (scan_angles <= self.fov_rad / 2.0)
        fov_indices = np.where(fov_mask)[0]
        sub_scan = np.clip(ranges[fov_indices], 0.0, 10.0)
        sub_angles = scan_angles[fov_indices]
        n_sub = len(sub_scan)

        # 2. Moving Average Filter (Remove sensor noise)
        kernel = np.ones(5) / 5.0
        smoothed = np.convolve(sub_scan, kernel, mode='same')
        proc_scan = np.copy(smoothed)

        # 3. UNC Chapel Hill Disparity Extender
        for i in range(n_sub - 1):
            diff = proc_scan[i + 1] - proc_scan[i]
            if abs(diff) > self.disparity_threshold:
                if diff > 0:
                    d_closer = proc_scan[i]
                    if d_closer > 0.05:
                        extend_angle = np.arctan(self.car_width_half / d_closer)
                        n_extend = int(np.ceil(extend_angle / angle_inc))
                        proc_scan[i + 1:min(n_sub, i + 1 + n_extend)] = np.minimum(
                            proc_scan[i + 1:min(n_sub, i + 1 + n_extend)], d_closer
                        )
                else:
                    d_closer = proc_scan[i + 1]
                    if d_closer > 0.05:
                        extend_angle = np.arctan(self.car_width_half / d_closer)
                        n_extend = int(np.ceil(extend_angle / angle_inc))
                        proc_scan[max(0, i - n_extend):i + 1] = np.minimum(
                            proc_scan[max(0, i - n_extend):i + 1], d_closer
                        )

        # 4. Bubble Inflation around Nearest Obstacle
        closest_idx = np.argmin(proc_scan)
        closest_dist = proc_scan[closest_idx]
        if 0.05 < closest_dist < 2.5:
            bubble_angle = np.arcsin(min(1.0, self.bubble_radius / closest_dist))
            n_bubble = int(np.ceil(bubble_angle / angle_inc))
            b_start = max(0, closest_idx - n_bubble)
            b_end = min(n_sub, closest_idx + n_bubble + 1)
            proc_scan[b_start:b_end] = 0.0

        # 5. Find Max Free Gap
        gaps = []
        in_gap = False
        gap_start = 0
        for i in range(n_sub):
            if proc_scan[i] >= self.gap_dist_thresh:
                if not in_gap:
                    in_gap = True
                    gap_start = i
            else:
                if in_gap:
                    in_gap = False
                    gaps.append((gap_start, i - 1))
        if in_gap:
            gaps.append((gap_start, n_sub - 1))

        if not gaps:
            best_idx = np.argmax(smoothed)
        else:
            widest_gap = max(gaps, key=lambda g: g[1] - g[0])
            g_start, g_end = widest_gap
            center_idx = (g_start + g_end) // 2
            deepest_in_gap = g_start + np.argmax(proc_scan[g_start:g_end + 1])
            best_idx = int(round(self.alpha * center_idx + (1.0 - self.alpha) * deepest_in_gap))
            best_idx = np.clip(best_idx, 0, n_sub - 1)

        raw_steer = sub_angles[best_idx]

        # 6. Lateral Tail Guard (Prevent tail-strike during sharp turning)
        lat_left = scan_angles > np.radians(65)
        lat_right = scan_angles < np.radians(-65)
        min_left = np.min(ranges[lat_left]) if np.any(lat_left) else 1.0
        min_right = np.min(ranges[lat_right]) if np.any(lat_right) else 1.0

        if raw_steer > 0.12 and min_left < 0.28:
            raw_steer = max(0.0, raw_steer * 0.45)
        elif raw_steer < -0.12 and min_right < 0.28:
            raw_steer = min(0.0, raw_steer * 0.45)

        # Steering Rate Slew Filter
        steer_diff = raw_steer - self.prev_steer
        steer = self.prev_steer + np.clip(steer_diff, -self.steer_max_rate, self.steer_max_rate)
        steer = np.clip(steer, -0.4189, 0.4189)
        self.prev_steer = steer

        # 7. Kinematic Threshold Lookahead Braking Curve
        abs_steer = abs(steer)
        target_dist = sub_scan[best_idx]
        fwd_mask = np.abs(sub_angles) < np.radians(14.0)
        fwd_dist = np.mean(sub_scan[fwd_mask]) if np.any(fwd_mask) else 3.0
        effective_clearance = min(fwd_dist, target_dist)

        # Curvature-based Speed Profile
        steer_deg = np.degrees(abs_steer)
        if steer_deg < 4.0:
            v_curv = self.max_speed
        elif steer_deg < 8.0:
            v_curv = self.max_speed * 0.88
        elif steer_deg < 13.0:
            v_curv = self.max_speed * 0.68
        elif steer_deg < 18.0:
            v_curv = self.max_speed * 0.48
        else:
            v_curv = self.min_speed

        # Lookahead Kinematic Deceleration
        if effective_clearance <= self.d_margin:
            v_brake = self.min_speed
        else:
            v_brake = np.sqrt(self.min_speed**2 + 2.0 * self.a_brake * (effective_clearance - self.d_margin))

        speed = max(self.min_speed, min(self.max_speed, min(v_curv, v_brake)))

        # Publish Drive Command
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = "base_link"
        drive_msg.drive.steering_angle = float(steer)
        drive_msg.drive.speed = float(speed)
        self.drive_pub.publish(drive_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DynamicApexGapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
