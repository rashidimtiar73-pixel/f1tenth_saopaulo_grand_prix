#!/usr/bin/env python3
"""
Chicane-tuned 10-Lap test
"""
import gym
import numpy as np
import time

def test_config(max_speed=8.8, min_speed=2.35, fwd_angle_deg=14.0, a_brake=5.8, d_margin=1.85):
    env = gym.make('f110_gym:f110-v0', 
                   map='/home/user/F1Tenth-Repository/src/f1tenth_gym_ros/maps/SaoPaulo_map', 
                   map_ext='.png', 
                   num_agents=1, 
                   disable_env_checker=True)

    sx = 0.0
    sy = 0.0
    stheta = -1.313

    obs, _, _, _ = env.unwrapped.reset(np.array([[sx, sy, stheta]]))
    num_beams = 1080
    fov = 4.7
    scan_angles = np.linspace(-fov/2.0, fov/2.0, num_beams)

    fov_rad = np.radians(135)
    bubble_radius = 0.45
    car_width_half = 0.31
    disparity_thresh = 0.38
    max_lidar_dist = 10.0
    gap_dist_thresh = 1.15
    alpha = 0.65
    steer_max_rate = 0.15
    prev_steer = 0.0

    lap_count = 0
    lap_times = []
    sim_time = 0.0
    lap_start_time = 0.0
    has_left_start = False
    collision = False
    max_spd = 0.0

    while lap_count < 10 and sim_time < 1200.0:
        scan = obs['scans'][0]

        fov_mask = (scan_angles >= -fov_rad / 2.0) & (scan_angles <= fov_rad / 2.0)
        fov_indices = np.where(fov_mask)[0]
        sub_scan = np.array(scan)[fov_indices]
        sub_angles = scan_angles[fov_indices]
        angle_inc = sub_angles[1] - sub_angles[0] if len(sub_angles) > 1 else 0.004

        sub_scan = np.clip(sub_scan, 0.0, max_lidar_dist)
        kernel = np.ones(5) / 5.0
        smoothed = np.convolve(sub_scan, kernel, mode='same')
        proc_scan = np.copy(smoothed)

        # Disparity Extender
        n_points = len(proc_scan)
        for i in range(n_points - 1):
            diff = proc_scan[i+1] - proc_scan[i]
            if abs(diff) > disparity_thresh:
                if diff > 0:
                    d_closer = proc_scan[i]
                    if d_closer > 0.05:
                        extend_angle = np.arctan(car_width_half / d_closer)
                        n_extend = int(np.ceil(extend_angle / angle_inc))
                        end_idx = min(n_points, i + 1 + n_extend)
                        proc_scan[i+1:end_idx] = np.minimum(proc_scan[i+1:end_idx], d_closer)
                else:
                    d_closer = proc_scan[i+1]
                    if d_closer > 0.05:
                        extend_angle = np.arctan(car_width_half / d_closer)
                        n_extend = int(np.ceil(extend_angle / angle_inc))
                        start_idx = max(0, i - n_extend)
                        proc_scan[start_idx:i+1] = np.minimum(proc_scan[start_idx:i+1], d_closer)

        # Bubble
        closest_idx = np.argmin(proc_scan)
        closest_dist = proc_scan[closest_idx]
        if 0.05 < closest_dist < 2.5:
            bubble_angle = np.arcsin(min(1.0, bubble_radius / closest_dist))
            n_bubble = int(np.ceil(bubble_angle / angle_inc))
            b_start = max(0, closest_idx - n_bubble)
            b_end = min(n_points, closest_idx + n_bubble + 1)
            proc_scan[b_start:b_end] = 0.0

        # Gap selection
        gaps = []
        in_gap = False
        gap_start = 0
        for i in range(n_points):
            if proc_scan[i] >= gap_dist_thresh:
                if not in_gap:
                    in_gap = True
                    gap_start = i
            else:
                if in_gap:
                    in_gap = False
                    gaps.append((gap_start, i - 1))
        if in_gap:
            gaps.append((gap_start, n_points - 1))

        if not gaps:
            best_idx = np.argmax(smoothed)
        else:
            widest_gap = max(gaps, key=lambda g: g[1] - g[0])
            g_start, g_end = widest_gap
            center_idx = (g_start + g_end) // 2
            deepest_in_gap = g_start + np.argmax(proc_scan[g_start:g_end + 1])
            best_idx = int(round(alpha * center_idx + (1.0 - alpha) * deepest_in_gap))
            best_idx = np.clip(best_idx, 0, n_points - 1)

        raw_steer = sub_angles[best_idx]

        # Lateral obstacle protection
        lat_left = scan_angles > np.radians(65)
        lat_right = scan_angles < np.radians(-65)
        min_left = np.min(scan[lat_left]) if np.any(lat_left) else 1.0
        min_right = np.min(scan[lat_right]) if np.any(lat_right) else 1.0

        if raw_steer > 0.12 and min_left < 0.28:
            raw_steer = max(0.0, raw_steer * 0.45)
        elif raw_steer < -0.12 and min_right < 0.28:
            raw_steer = min(0.0, raw_steer * 0.45)

        steer_diff = raw_steer - prev_steer
        steer = prev_steer + np.clip(steer_diff, -steer_max_rate, steer_max_rate)
        steer = np.clip(steer, -0.4189, 0.4189)
        prev_steer = steer

        # Speed profile
        abs_steer = abs(steer)
        target_dist = sub_scan[best_idx]
        fwd_mask = np.abs(sub_angles) < np.radians(fwd_angle_deg)
        fwd_dist = np.mean(sub_scan[fwd_mask]) if np.any(fwd_mask) else 3.0
        effective_clearance = min(fwd_dist, target_dist)

        steer_deg = np.degrees(abs_steer)
        if steer_deg < 4.0:
            v_curv = max_speed
        elif steer_deg < 8.0:
            v_curv = max_speed * 0.88
        elif steer_deg < 13.0:
            v_curv = max_speed * 0.68
        elif steer_deg < 18.0:
            v_curv = max_speed * 0.48
        else:
            v_curv = min_speed

        if effective_clearance <= d_margin:
            v_brake = min_speed
        else:
            v_brake = np.sqrt(min_speed**2 + 2.0 * a_brake * (effective_clearance - d_margin))

        speed = max(min_speed, min(max_speed, min(v_curv, v_brake)))
        max_spd = max(max_spd, speed)

        obs, reward, done, info = env.unwrapped.step(np.array([[steer, speed]]))
        sim_time += 0.01

        curr_x = obs['poses_x'][0]
        curr_y = obs['poses_y'][0]
        curr_pos = np.array([curr_x, curr_y])

        if obs['collisions'][0]:
            print(f"❌ Collision at sim_time={sim_time:.2f}s, pos=({curr_x:.2f}, {curr_y:.2f})!")
            collision = True
            break

        dist_to_start = np.linalg.norm(curr_pos - np.array([sx, sy]))
        if dist_to_start > 15.0:
            has_left_start = True

        if has_left_start and dist_to_start < 2.2:
            lap_time = sim_time - lap_start_time
            if lap_time > 10.0:
                lap_count += 1
                lap_times.append(lap_time)
                lap_start_time = sim_time
                has_left_start = False
                print(f"🏁 LAP {lap_count}/10 COMPLETED! Time: {lap_time:.3f} s (Best: {min(lap_times):.3f} s)")

    print(f"\nResult: Laps {lap_count}/10 | Collision: {collision} | Top: {max_spd*3.6:.1f} km/h")
    if lap_times:
        print(f"Fastest: {min(lap_times):.3f} s | Average: {np.mean(lap_times):.3f} s")
    return lap_count, lap_times, collision

if __name__ == '__main__':
    test_config()
