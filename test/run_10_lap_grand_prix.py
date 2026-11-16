#!/usr/bin/env python3
"""
Full 10-Lap Grand Prix Verification for the Champion Controller (Sao Paulo Track).
"""
import gym
import numpy as np
import time
import os
import csv

def run_10_laps():
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

    max_speed = 9.5
    min_speed = 2.65
    a_brake = 6.2
    d_margin = 1.75
    fov_rad = np.radians(135)
    bubble_radius = 0.44
    car_width_half = 0.31
    disparity_thresh = 0.38
    max_lidar_dist = 10.0
    gap_dist_thresh = 1.15
    alpha = 0.65
    steer_max_rate = 0.16
    prev_steer = 0.0

    lap_count = 0
    lap_times = []
    sim_time = 0.0
    lap_start_time = 0.0
    has_left_start = False
    collision = False
    max_speed_reached = 0.0

    print("=================================================================")
    print("🚀 STARTING OFFICIAL 10-LAP GRAND PRIX VERIFICATION RUN")
    print(f"Track: Sao Paulo (Interlagos) | Target Laps: 10")
    print(f"Max Speed: {max_speed} m/s ({max_speed*3.6:.1f} km/h) | Min Speed: {min_speed} m/s")
    print("=================================================================\n")

    start_wall = time.time()

    while lap_count < 10 and sim_time < 1200.0:
        scan = obs['scans'][0]

        # 1. FOV windowing
        fov_mask = (scan_angles >= -fov_rad / 2.0) & (scan_angles <= fov_rad / 2.0)
        fov_indices = np.where(fov_mask)[0]
        sub_scan = np.array(scan)[fov_indices]
        sub_angles = scan_angles[fov_indices]
        angle_inc = sub_angles[1] - sub_angles[0] if len(sub_angles) > 1 else 0.004

        # 2. Preprocess & Smoothing
        sub_scan = np.clip(sub_scan, 0.0, max_lidar_dist)
        kernel = np.ones(5) / 5.0
        smoothed = np.convolve(sub_scan, kernel, mode='same')
        proc_scan = np.copy(smoothed)

        # 3. Disparity Extender
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

        # 4. Bubble at closest obstacle
        closest_idx = np.argmin(proc_scan)
        closest_dist = proc_scan[closest_idx]
        if 0.05 < closest_dist < 2.5:
            bubble_angle = np.arcsin(min(1.0, bubble_radius / closest_dist))
            n_bubble = int(np.ceil(bubble_angle / angle_inc))
            b_start = max(0, closest_idx - n_bubble)
            b_end = min(n_points, closest_idx + n_bubble + 1)
            proc_scan[b_start:b_end] = 0.0

        # 5. Gap Selection
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

        # Steering rate limit
        steer_diff = raw_steer - prev_steer
        steer = prev_steer + np.clip(steer_diff, -steer_max_rate, steer_max_rate)
        steer = np.clip(steer, -0.4189, 0.4189)
        prev_steer = steer

        # 6. Kinematic Threshold Braking Speed Profile
        abs_steer = abs(steer)
        target_dist = sub_scan[best_idx]
        fwd_mask = np.abs(sub_angles) < np.radians(6.5)
        fwd_dist = np.mean(sub_scan[fwd_mask]) if np.any(fwd_mask) else 3.0
        effective_clearance = min(fwd_dist, target_dist)

        # Curvature speed mapping
        steer_deg = np.degrees(abs_steer)
        if steer_deg < 4.0:
            v_curv = max_speed
        elif steer_deg < 8.0:
            v_curv = max_speed * 0.90
        elif steer_deg < 13.0:
            v_curv = max_speed * 0.70
        elif steer_deg < 18.0:
            v_curv = max_speed * 0.50
        else:
            v_curv = min_speed

        # Kinematic braking profile
        if effective_clearance <= d_margin:
            v_brake = min_speed
        else:
            v_brake = np.sqrt(min_speed**2 + 2.0 * a_brake * (effective_clearance - d_margin))

        speed = max(min_speed, min(max_speed, min(v_curv, v_brake)))
        if speed > max_speed_reached:
            max_speed_reached = speed

        # Step simulation
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
                print(f"🏁 LAP {lap_count}/10 COMPLETED! Time: {lap_time:.3f} s | Fastest So Far: {min(lap_times):.3f} s")

    elapsed = time.time() - start_wall
    print("\n=================================================================")
    print("🏆 10-LAP GRAND PRIX VERIFICATION COMPLETE")
    print(f"Laps Completed: {lap_count}/10")
    print(f"Collision Free: {not collision}")
    if lap_times:
        print(f"🥇 Fastest Lap: {min(lap_times):.3f} s")
        print(f"⏱️ Average Lap: {np.mean(lap_times):.3f} s")
        print(f"🏎️ Top Speed: {max_speed_reached*3.6:.1f} km/h ({max_speed_reached:.2f} m/s)")
    print(f"Real Computation Time: {elapsed:.2f} s")
    print("=================================================================\n")

    # Save to CSV
    data_dir = '/home/user/.gemini/antigravity/scratch/f1tenth_saopaulo_grand_prix/data'
    os.makedirs(data_dir, exist_ok=True)
    csv_path = os.path.join(data_dir, 'champion_lap_times.csv')
    with open(csv_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['Lap', 'Lap_Time_Seconds', 'Fastest_So_Far_Seconds', 'Collisions', 'Top_Speed_kmh'])
        best = float('inf')
        for i, t in enumerate(lap_times, 1):
            best = min(best, t)
            writer.writerow([i, f"{t:.3f}", f"{best:.3f}", 0, f"{max_speed_reached*3.6:.1f}"])
    print(f"Results saved to {csv_path}")

    return lap_count, lap_times, collision

if __name__ == '__main__':
    run_10_laps()
