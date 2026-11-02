# JetBot Pro: findings for the report (session of 2026-11-01 to 2026-11-02)

Source: the working session between the user and Claude Code. Each finding carries a confidence tag:
**[measured]** observed directly, **[derived]** computed from measurements, **[hypothesis]** a plausible explanation that was not tested, **[corrected]** an earlier claim that was found wrong.
Nothing here claims the robot navigates reliably. See section 9 for what remains unproven.

---

## 1. Platform and environment

- **Robot:** NVIDIA Jetson Nano 4 GB on a JetBot Pro chassis. **OS/middleware:** Ubuntu with ROS 1 Melodic. Catkin workspace `/home/nvidia/catkin_ws`, package `jetbot_pro` in `src/jetbot_pro`. Jetson git is 2.17.1 (no `git restore`, no `git branch --show-current`).
- **Serial devices [measured]:** chassis `/dev/ttyACM0`, RPLidar `/dev/ttyACM1`, both 115200 baud. Enumeration order could swap; a udev rule is advisable.
- **Network:** the Jetson's address changed between days (192.168.68.108, then .107). Do not hard-code it; SSH key `~/.ssh/jetson_claude` needs loading into the ssh-agent (passphrase-protected).
- **Dev machine:** Apple Silicon Mac, no ROS installed. Everything ROS-related was run on the Jetson. Local work was static checks (XML/YAML validation), Python unit tests for the odometry node, and analysis scripts.
- **Software versions seen:** rplidar_ros 1.7.0; slam_gmapping; AMCL; move_base with GlobalPlanner and TEB local planner; map_server; teleop_twist_keyboard 1.0.0.
- **Repository:** fork `yeses03/jetbot_pro` (upstream `waveshare/jetbot_pro`). Work branch `claude/jetbot-diagnostics`.

## 2. Frames, topics and nodes (as found running)

- Frames: `map`, `odom`, `base_footprint`, `laser_frame`, plus `base_imu_link`. Static TFs publish on `/tf`, not `/tf_static` **[measured]**.
- Chain: `/jetbot` (chassis driver) publishes `/odom_raw` (~50 Hz) -> `robot_pose_ekf` -> `/odom_combined` -> `odom_ekf_node` (`scripts/odom_ekf.py`) -> `/odom` (~24 Hz). `/rplidarNode` -> `/scan` (~6.6–7.2 Hz). The `odom -> base_footprint` TF comes from `robot_pose_ekf`; the driver's own `publish_odom_transform` is false **[measured]**.
- Nodes in the final stack: `/jetbot`, `/rplidarNode`, `/robot_pose_ekf`, `/odom_ekf_node`, `/base_link_to_laser`, `/base_footprint_to_imu`, `/map_server`, `/amcl`, `/move_base` (GMapping and the camera only in the mapping stack).
- **Do not launch duplicate chassis or lidar nodes:** `slam.launch`, `nav.launch` and `slam_nav.launch` each include `jetbot.launch` and `lidar.launch`.

## 3. Chassis driver facts (`src/jetbot.cpp`)

- Wheel odometry is **computed in the robot's firmware, not in ROS.** The driver only reads x, y, yaw (serial bytes 22–27) and dx, dy, dyaw (bytes 28–33). The firmware is not in the repo **[measured from code]**.
- The driver sends `linear_correction` and `angular_correction` to the firmware at startup (`SetParams`, message type 0x13); both are 1.0 in `jetbot.launch`. They are the only odometry calibration knobs in software.
- A 1 s host-side watchdog zeroes the commanded velocity. The main loop has no sleep, so one thread sat at about 99.9% CPU; the 200 Hz `ros::Rate` fix exists as an unbuilt local patch **[measured; effect on navigation not shown]**.
- `cmd_callback` contains `y = msg.linear.x`, a likely upstream quirk **[not investigated]**.
- **There is no software e-stop and no velocity mux.** Stops are the power switch, the cancel topic, and the 1 s watchdog (which a continuously publishing `move_base` defeats).

## 4. Odometry and TF fixes (deployed)

- **Covariance [measured]:** `/odom` pose covariance grew as 1e6 × the number of messages (reaching ~1.5e10). The fix caps the unobserved yaw/roll/pitch/z entries at 1e6, copies the twist from `/odom_raw` while fresh (0.5 s), and sets `frame_id` to `odom` without the leading slash. File: `scripts/odom_ekf.py`, committed in `bd37a5f` with 8 unit tests (`test/test_odom_ekf.py`, run by path because `test/` shadows the stdlib package). Python 2 compatible; the tests need Python 3 locally.
- **Lidar yaw [measured]:** the scan's 0° points to the robot's **rear**. Evidence: a monitor in front of the camera appeared at scan bearing 180°; a cushion stack on the robot's left appeared at about −106°, which also rules out a mirrored scan. Correct static TF is `base_footprint -> laser_frame` yaw **3.14** (the committed Waveshare value; the Jetson's experimental yaw 0.0 was 180° wrong). Applied in `launch/lidar.launch` line 13 on the Jetson working tree. [corrected] An early claim that a near arc was a robot self-return was wrong; a line fit showed a real wall.
- Jetson-vs-GitHub drift: the Jetson `main` tracks upstream (Waveshare) and has staged experimental edits (`imu_used=false`, smaller costmap radii, TEB `min_obstacle_dist` 0.05) and a lidar.launch whose index differs from the working tree. These were deliberately **not** ported or reset; the experimental values were not validated.

## 5. Mapping (GMapping)

- Launch: `slam.launch` (includes jetbot, lidar, camera, gmapping). `gmapping.launch` defaults: delta 0.05, maxUrange 8.0, maxRange 12.0, 8 particles, linearUpdate 0.05, angularUpdate 0.0436, map_update_interval 2.0.
- **Bounds [derived from upstream source, matches a measured map]:** `xmin/ymin/xmax/ymax` set the *initial* map size, cells = floor(ceil(2B/delta)/32)*32. The shipped ±50 m gave a 1984×1984 map (3.9 M cells, 98% unknown). GMapping still enlarges the map in 10 m steps if scan endpoints leave the box. We ran **±8 m (320×320)** by setting `rosparam` and respawning `/slam_gmapping` (it has `respawn=true`, so `rosnode kill` restarts it).
- The map origin is defined by the robot pose at the first scan: **GMapping must be started with the robot at the intended start pose, stationary.**
- **Run result [measured]:** a teleoperated loop of the room (`teleop_twist_keyboard`, speed 0.1, turn 0.3) closed to within about **11 cm** of the start by GMapping's own pose, while raw wheel odometry had drifted about **0.9 m**. Saved with `map_saver` to a unique name (never overwrite existing maps): `livingroom_20261102_0830.pgm/.yaml`, 320×320 at 0.05 m, origin (−8, −8). The user judged it acceptable by eye; no tape-measured wall check was done.
- Map defects found later: the open doorway leaks a long free-space corridor; walls are ragged in places; **8 false occupied cells at the start position** (noise from the mapping run) made the planner think the start pose was in collision. Variants: `_cropped` (corridor set to unknown) and `_start_cleaned` (the 8 cells cleared where the live scan showed free space).

## 6. Localization (AMCL)

- Launch: `launch/localize.launch` (new): chassis, lidar, `map_server`, AMCL. `amcl.launch` is unchanged; overrides are launch args that set `/amcl/*` params after the include. AMCL starts from `initial_pose_x/y/a` (default 0,0,0 = the mapping start line).
- **Findings [measured]:**
  - AMCL **lags during motion** (updates only after 0.2 m / 0.5 rad of motion by default) and corrects in jumps; at rest, repeated `/request_nomotion_update` calls (a no-motion service, it does not move the robot) converge the pose over ~10–30 updates.
  - If the robot is lifted or moved by hand, odometry and AMCL do not know: they are "kidnapped". After carrying, relaunch with a correct initial pose.
  - The shipped laser model weights (`laser_z_hit` 0.5, `laser_z_rand` 0.5) are unusual and flat; ROS defaults are 0.95 / 0.05. Changed to 0.9 / 0.1 and the update thresholds to 0.05 m / 0.1 rad, laser beams 30 to 60.
- **Two real localization failures [measured by scan-to-map fit]:**
  1. After a 2 m navigation leg AMCL's x was about **0.35–0.4 m too far ahead** (scan fit best at x≈1.43, AMCL 1.80). The real rear wall, drawn at the wrong map position, made the goal cell lethal and the return goal failed. Re-seeding with `initial_x:=1.45` and the new weights brought AMCL within 2 cm of the scan fit **at rest**.
  2. During a 180° in-place turn at navigation speed AMCL's heading ended about **30° off** (scan fit 192°, AMCL 161°); wheel odometry reported −221° for a turn of about −168°, a rotation over-read of about 30%. Result: slanted-wall costmap, "no valid plan", safe abort.
- **Open question [measured, unexplained]:** after re-seeding AMCL at (1.45, 0.15), the scan fit still put the best y at about 0.30, 0.12 m from AMCL's 0.18, while x agreed within 2 cm. Either AMCL has a lateral error the walls do not correct (the room is a long box), or the fit's y estimate is weak. Not resolved.
- **Ground-truth method that worked:** slide the live `/scan` over the saved PGM and maximise the fraction of scan points within 5 cm of a map wall (about 80–89% at the true pose, 31–58% at a wrong one). Hand tape measurements and AMCL itself were each biased at times; the scan fit was consistent with the rear-wall range and with the costmap evidence.

## 7. Odometry calibration (the main unresolved problem)

| Run | `linear_correction` | Odometry | Reference | Ratio odom/ref | Reference quality |
|---|---|---|---|---|---|
| Teleop forward, run 1 | 1.0 | 1.507 m | tape 1.98 m | 0.76 | tape |
| Teleop reverse | 1.0 | 1.518 m | tape ≈2.0 m | ≈0.76 | tape |
| Teleop forward, run 2 | 1.0 | 1.496 m | tape 1.97–2.0 m | ≈0.75 | tape |
| Teleop forward | 1.3 | 1.694 m | tape ≈1.97 m | 0.86 | tape |
| Teleop forward | 1.5 | 2.40 m | AMCL 2.08, tape 1.97 | 1.15–1.22 | AMCL (later shown biased) |
| Teleop forward | 1.7 | 2.29 m | tape ≈2.1 m | ≈1.09 | tape |
| Teleop forward | 1.4 | 1.99 m | AMCL 2.00; user's tape 2.4 | ≈1.0 or 0.83 | conflicting |
| Navigation leg 2 | 1.4 | ≈1.60 m | scan fit ≈1.08 m | ≈1.48 | scan fit |
| Spin, teleop 0.3 rad/s | 1.0 | 181° and 160° | ≈180° by eye | ≈0.9–1.0 | eye |
| Nav turn at 0.6 rad/s | 1.4 | 221° | scan fit ≈168° | ≈1.31 | scan fit |

- **Findings:** at 1.0 the wheel odometry under-reports distance by about 24% (consistent across three clean tape runs); rotation was about right. `linear_correction` changes **both** odometry and the **real robot speed** (real speed at 0.10 m/s command fell to roughly half at 1.4–1.7), and it also scales odometry rotation. The effect is not a clean 1:1 multiplier.
- **[corrected] The 1.4 choice was a mistake:** it relied on AMCL (biased) and an inconsistent tape. The scan-fit evidence suggests the right value is nearer **1.1–1.2** with `angular_correction` about **0.9**. **[hypothesis]** until measured.
- Odometry quality depends on speed; 0.1 m/s teleop numbers do not transfer to 0.2 m/s navigation.

## 8. Navigation (move_base, TEB)

- **Configuration set `config/living_room/`** (copy of `config/diff` with changes; the Jetson's experimental `config/diff` edits are untouched):

| Parameter | Original | Used |
|---|---|---|
| TEB `max_vel_x` (commanded) | 0.7 | 0.22 |
| `max_vel_x_backwards` | 0.35 | 0.15 (0.10 was ≤ `penalty_epsilon`, undefined) |
| `max_vel_theta` | 2.5 | 0.6 |
| `acc_lim_x` / `acc_lim_theta` | 1.2 / 1.5 | 0.3 / 0.6 |
| `min_obstacle_dist` | 0.20 | 0.10 |
| `xy_goal_tolerance` / `yaw_goal_tolerance` | 0.2 / 0.5 | 0.15 / 0.6 |
| `footprint_model` | `radius: 0.12` with **no `type`** | `type: "circular"`, radius 0.12 |
| `allow_unknown` (global costmap and GlobalPlanner) | true | false |
| `controller_frequency` | 4 Hz | 6 Hz |
| Rotate-in-place recovery | on | off |

- **Footprint bug [measured]:** the original TEB `footprint_model` had no `type`, so TEB defaulted to a point footprint (inscribed radius 0.0) and ignored the 0.12 m radius. Costmap inscribed radius is about 0.127 m.
- Costmap facts: global costmap loads the static map and, because no `plugins` list is given, the pre-Hydro default plugin set (with a live obstacle layer); local costmap is a 3×3 m rolling window at 0.05 m. `/scan_2` and sonar sources in `costmap_common_params.yaml` are unused.
- **`navigate.launch` (new, safe by default):** `move_base` output is remapped to `/cmd_vel_dry`, which the chassis does not read. Real motion needs `cmd_vel_topic:=cmd_vel`. A dry run (goal 1.0 m ahead) produced a 1.17 m global plan and bounded commands (0–0.22 m/s, 0–0.42 rad/s), then aborted "oscillating", which is the expected dry-mode artefact.
- **Real goals (supervised, power switch in hand):**
  1. Goal 1 failed first: "trajectory not feasible", because of the 8 false map cells at the start. After cleaning the map, goal 1 (0.5 m ahead) was reached in about 5 s; user measured about **35 cm** moved; AMCL settled at (0.29, 0.08).
  2. Goal 2 (map (2.0, 0.1)) reached in about 8 s of driving (odometry speed ≈0.19 m/s). AMCL said 1.80 m (settled, stable) but the scan fit says the robot was at about **x = 1.45**. The robot stopped as soon as it was inside the 0.25 m tolerance circle, so accuracy was tolerance-limited as well as pose-limited.
  3. Return goal (0, 0): first aborted "no plan" (AMCL x error, see section 6); after re-seeding and retuning, the robot turned about 168–180° and drove about 0.5 m, then aborted "failed to find a valid plan" because AMCL's heading had drifted 30°. It stopped safely. The battery then died.
- **Time per leg [measured, small sample]:** about 1.6 m in 8–10 s of driving at the 0.22 m/s cap, plus a few seconds to start and stop; a 3 m leg would be roughly 15–20 s.

## 9. Not proven / open issues

- Odometry scale at navigation speed and the correct `linear_correction` / `angular_correction` values.
- AMCL yaw and x accuracy **while moving and turning**; accuracy of any goal.
- The return trip and any turn of about 180° at navigation speed.
- Behaviour near the open doorway (unknown space is disallowed in the planner, but the map edge there is open).
- Whether `linear_correction` persists in firmware across a power cycle (not tested).
- The `jetbot.cpp` loop-rate patch (unbuilt) and whether it affects navigation; the GUI integration (read-only Tk/rosbridge dashboard exists in `gui/`, not deployed); the Git reconciliation on the Jetson; the `y = msg.linear.x` quirk.
- Nothing from this work is committed or pushed except the odometry fix (`bd37a5f`).

## 10. Safety and operations findings

- **Hard rules followed:** no `/cmd_vel` publishing by the agent; the user alone drives teleop and sends goals (the permission system also blocks the agent from publishing nav goals); no nav goal without a human at the power switch; no deploy, commit or push without explicit approval; original maps and Jetson git state preserved.
- **Monitor:** a read-only node (`mapping_monitor_amcl.py`) subscribed to `/scan`, `/odom`, `/cmd_vel`, `/map_metadata`, `/rosout` and TF; it alarmed and exited when anything came within 0.17 m of the lidar for 3 scans, when scans or odometry went stale for 4 s, when `map->odom` jumped, or when AMCL died. It suppressed stationary rear returns (the charger). Limitation: an alarm stops the monitor but not the robot, and a person standing near the robot causes false alarms.
- **Operational hazards observed:** the battery drains fast and died twice mid-test (it changes speed and ruins comparisons); the charger cable at the rear is a snag/obstacle hazard; accidental diagonal teleop keys (`o`, `u`) arced the robot toward the wall; speed keys (`q w x z e c`) change commanded speed mid-run; lifting the robot invalidates AMCL; `pgrep -f` patterns can match their own shell and kill the session; a ROS master restart kills teleop.
- **Clearances in the test room (lidar centre to wall):** about 0.30 m on the right at the start line, 0.44–0.46 m behind, 0.88–1.1 m left, front open about 3.8 m. The chassis is about 0.19–0.20 m wide (lidar over the wheel axis), so right-side body clearance is about 0.2 m. The robot is within a few centimetres of the wall in tight cases, so a spin or an arc there is a collision risk.

## 11. Replication checklist (what to do, in order)

1. Check the Jetson address, load the SSH key, source `/opt/ros/melodic/setup.bash` and `~/catkin_ws/devel/setup.bash`.
2. Confirm the deployed fixes: `launch/lidar.launch` yaw 3.14 and the fixed `scripts/odom_ekf.py`. Never relaunch the lidar TF with yaw 0.
3. Charge the battery fully. Charger out and clear of the wheels before any motion.
4. Place the robot on the start line facing the fridge, stationary, on battery. Map with `slam.launch` (±8 m bounds, no duplicate nodes); teleop slowly (speed 0.1, turn 0.3, `i , j l k` only); save with `map_saver` to a **new** name.
5. Remove the doorway corridor and any start-area noise in a **copy** of the map, guided by a live scan.
6. Launch `localize.launch` / `navigate.launch` with the correct initial pose and safe default; verify lidar yaw, odometry at 0, AMCL at the start pose, `/cmd_vel` with no unexpected publishers.
7. **Calibrate odometry** using scan-to-map fits as ground truth: one straight drive and one slow spin at the speeds navigation will use; adjust `linear_correction` and `angular_correction` one at a time, re-test each.
8. Check AMCL heading and position after each leg (scan-to-map fit), use `request_nomotion_update` at rest.
9. Dry-run `move_base`; then supervised real goals, shortest first, with the power switch in hand and the cancel command ready: `rostopic pub -1 /move_base/cancel actionlib_msgs/GoalID '{}'`.

## 11b. Tools added after the session (offline, no robot motion)

- `tools/scan_dump.py` runs on the Jetson (read-only) and prints a merged scan as robot-frame JSON; `tools/scan_map_fit.py` slides it over a saved map and reports the best pose and the fit at a guessed pose. On the session's data it reproduces the 0.39 m x error (best x 1.45 against AMCL's 1.80).
- The dashboard (`gui/`) now has a *Pose vs scan* chip using the same fit, tested on real scans (see its README). It has never been run against the live robot: rosbridge is not installed on the Jetson and installing it needs a `sudo` password.
- Commits are local on branch `claude/jetbot-diagnostics`, not pushed.

## 12. Deliverable files (all in the repo working tree, uncommitted unless stated)

`scripts/odom_ekf.py` and `test/test_odom_ekf.py` (committed); `launch/localize.launch`, `launch/navigate.launch`, `config/living_room/*.yaml`; maps in `~/jetbot_maps` (`livingroom_20261102_0830`, `_cropped`, `_start_cleaned`); `tools/jetson_readonly_diag.sh`; `gui/` (read-only dashboard, 15/15 tests pass with `roslibpy`, `numpy`, `pillow`, `websockets`); `src/jetbot.cpp` (local, held loop-rate patch).
