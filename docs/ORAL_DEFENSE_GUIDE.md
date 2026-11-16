# F1Tenth Project Defense Cheat Sheet — Multiplier ×1.0 Strategy
**Course:** Autonomous Vehicles (Tongmyong University)  
**Instructor:** Prof. Nabih Pico, Ph.D.  
**Track Assigned:** Sao Paulo (Interlagos Circuit) — Group 5  
**Objective:** Maintain 100% grade preservation ($\times 1.0$) during the 1-on-1 oral interview.

---

## 🎯 Top 5 Questions Prof. Pico Will Ask & Exact Answers

### Q1: "How does your Follow-the-Gap algorithm choose its steering angle, and why doesn't your car zigzag on straightaways?"
* **Core Answer:**
  > "Standard Follow-the-Gap algorithms steer towards the single deepest LiDAR point in the gap. On long straights, sensor noise and slight perspective changes cause the deepest point to rapidly jump between the left and right sides of the corridor, causing severe zigzag oscillations (as discussed in Class Slide 40).
  > To solve this, my controller calculates both the geometric midpoint of the widest gap:
  > $$i_{center} = \frac{g_{start} + g_{end}}{2}$$
  > and the deepest point $i_{deep}$, then blends them with an $\alpha = 0.65$ weighting:
  > $$i_{steer} = 0.65 \cdot i_{center} + 0.35 \cdot i_{deep}$$
  > Furthermore, I implemented a steering slew rate limiter ($\Delta\delta \le 0.15\text{ rad/step}$). This keeps the car centered on straights while smoothly transitioning into turns with zero wobble."

---

### Q2: "What is the Disparity Extender, and what is the exact formula you used to project the bubble?"
* **Core Answer:**
  > "When an obstacle edge creates a depth jump (disparity) greater than $\Delta d > 0.38\text{ m}$ between adjacent laser beams, the vehicle risks clipping that inside corner with its chassis.
  > Following the UNC Chapel Hill Disparity Extender algorithm, we treat the closer point $d_{closer}$ as a physical obstacle edge and calculate how many laser rays must be artificially masked to guarantee clearance for our car's width ($W_{car} = 0.31\text{ m}$ half-width):
  > $$\theta_{extend} = \arctan\left(\frac{W_{half}}{d_{closer}}\right)$$
  > $$N_{extend} = \left\lceil \frac{\theta_{extend}}{\Delta\theta} \right\rceil$$
  > We then clamp the next $N_{extend}$ beams to $d_{closer}$. This ensures the widest gap calculation never tries to squeeze the car through an unnavigable space."

---

### Q3: "How did you achieve a sub-88 second lap time without colliding into the walls in Sao Paulo's chicanes?"
* **Core Answer:**
  > "Standard controllers use a fixed low speed (2.5–3.5 m/s) out of fear of crashing, which produces slow 130–150 second laps. If you simply raise the speed to 8 m/s without predictive braking, the car understeers and crashes into corner walls.
  > I implemented **Kinematic Threshold Lookahead Braking**:
  > 1. The controller inspects a $14^\circ$ forward cone and measures effective clearance up to $10\text{ m}$ ahead.
  > 2. Using the constant-deceleration kinematic equation:
  >    $$v_{brake} = \sqrt{v_{min}^2 + 2 \cdot a_{brake} \cdot (d_{clearance} - d_{margin})}$$
  >    with $a_{brake} = 5.8\text{ m/s}^2$ and $d_{margin} = 1.85\text{ m}$.
  > 3. This allows the car to reach **$8.8\text{ m/s}$ ($31.68\text{ km/h}$)** down the main straights, and autonomously decelerate to **$2.35\text{ m/s}$** at the exact threshold braking point right before entering the chicanes.
  > 4. In addition, I implemented a **Lateral Tail Guard** that monitors side beams ($|\theta| \in [65^\circ, 85^\circ]$). If lateral clearance drops below $0.28\text{ m}$, steering magnitude is attenuated by $55\%$ to prevent the car's rear from hitting the inside wall."

---

### Q4: "How does your Lap Counter and Lap Timer work in code?"
* **Core Answer:**
  > "In `dynamic_apex_gap_node.py`, I subscribe to `/ego_racecar/odom`.
  > 1. **Start Coordinate:** The track starts at $(x=0.0, y=0.0)$ with orientation $-1.313\text{ rad}$.
  > 2. **Arming Phase:** To prevent false triggering right after launch, the gate only arms after the vehicle travels more than $15\text{ m}$ away from the start line (`has_left_start = True`).
  > 3. **Gate Crossing:** When the car returns within $2.2\text{ m}$ of $(0.0, 0.0)$, the lap counter increments (`lap_count += 1`), the elapsed time is recorded using `self.get_clock().now()`, and the data is written directly to `champion_lap_times.csv`.
  > 4. A debounce condition ensures no double-counting can occur.
  > 5. A 3D RViz Marker (`/lap_hud_marker`) publishes the current lap, live running timer, and best lap time directly into RViz."

---

### Q5: "What are the limitations of a purely reactive controller like Follow-the-Gap compared to trajectory-tracking controllers (like Pure Pursuit or MPC)?"
* **Core Answer:**
  > "A reactive controller only makes decisions based on the current instantaneous LiDAR scan (local horizon). It does not have a global map or know the upcoming track curvature until the sensor sees it.
  > In contrast, an MPC or Pure Pursuit controller follows a precomputed, globally optimized minimum-time racing line.
  > However, my enhanced Follow-the-Gap controller bridges this gap by incorporating predictive kinematic braking and disparity extension, enabling near-optimal racing line speeds while retaining the dynamic obstacle avoidance adaptability of reactive methods."
