# JetBot Pro project: source material for (A) a setup manual and (B) a project report

Written for: a documentation-writing assistant who will turn this into two documents. It is raw, accurate source material, not the finished text.
Everything here comes from the real work done 2026-11-01 to 2026-11-04. Where something was NOT verified on the physical robot it says so; do not
present those items as working. Repository: https://github.com/yeses03/jetbotpro (main). Detailed companion files in the repo are listed in section 9.

Suggested documents:
- **A. Setup manual**: how to set up this JetBot from scratch to "navigates a mapped room, controlled from a Mac GUI". Use section 0 (status) and sections 1 to 4 and 8.
- **B. Project report**: how it was built, with the major errors and the fixes. Use section 0 (status), sections 2, 5 and 6 (timeline).

---

## 0. PROJECT STATUS (read this first; use it to label every feature in both documents)

Legend: **DONE** = built and tested on the physical robot or, for software-only parts, by automated tests, and confirmed.
**PARTIAL** = built and tested only against fakes or a spare setup, not yet run end to end on the live robot.
**PLANNED** = not built; an intention or idea only.

### 0.1 Current implementation

| Area | Feature | Status | Evidence / note |
|---|---|---|---|
| Robot base | LiDAR transform fixed (yaw 3.14) and deployed | DONE | Scan matches the map after the fix |
| Robot base | Odometry republisher rewritten (`odom_ekf.py`) with a test | DONE | Deployed after approval, committed |
| Robot base | Odometry calibration, linear 1.15 and angular 0.9 | DONE | Measured with the scan-to-map fit; used in every later run |
| Mapping | Living-room map built and saved (with cleaned and walled copies) | DONE | `~/maps/livingroom_20261102_0830*` on the robot |
| Mapping | Whole far, furnished end of the room mapped well enough to navigate | NOT VERIFIED | Needs a re-map |
| Localisation | AMCL on the saved map with a seeded start pose | DONE | Works for supervised runs; always wrong after a hand-carry |
| Localisation | Re-seeding AMCL by relaunch, or in the GUI ("Check pose from scan" then "Apply to AMCL") | DONE | Used repeatedly |
| Localisation | Lowered AMCL motion noise (0.12/0.10/0.20/0.10, sigma_hit 0.10) | PARTIAL | Deployed, never run in motion |
| Navigation | `move_base` with GlobalPlanner and TEB, tuned for a 20 cm robot | DONE | Several goals reached; a long route reached legs 1 to 3 (leg 4 aborted in the tight start corner) |
| Navigation | Default dry-run launch (`cmd_vel_dry`) so a mistaken start cannot move the robot | DONE | |
| GUI console (Mac) | Live map, robot pose, LiDAR overlay, status cards | DONE | User confirmed "works perfectly" |
| GUI console | Click-to-go with the MOTION switch and safety gate | DONE | |
| GUI console | Saved named places (per map), routes, planned-path overlay | DONE | |
| GUI console | Pose-versus-scan check and relocalise buttons | DONE | |
| GUI console | Keyboard and button teleop with caps, LiDAR bumper and stop on focus loss | DONE | |
| GUI console | 146 automated tests (no robot needed), passing 15 times in a row | DONE | Flaky test fixed 2026-11-04 |
| Network security | rosbridge firewall rule plus SSH tunnel | DONE (manual) | Rule is lost on reboot until the systemd unit is installed |
| Robot manager | Robot-side service: start/stop navigation or mapping, map library, save/rename/delete/restore | PARTIAL | 33 tests pass; read-only endpoints, rename, delete, restore tried on the robot on a spare port; start/stop and save never run live |
| Robot manager | systemd install (firewall + manager units) | PARTIAL | Written and dry-run tested; NOT installed (needs the user's sudo) |
| GUI Robot tab | Connect through an SSH tunnel opened by the app, start navigation, map a new area, stop, map library | PARTIAL | Tested through the real window against fakes; never against the live robot |
| Docs | GUI README, robot setup guide, manager README, findings report, this handoff | DONE | |
| Git | 77 fine-grained commits on `main` of `yeses03/jetbotpro`, 8 milestone tags, rollback point `jetbot/07-console-stable` | DONE | Also on the fork `yeses03/jetbot_pro` |

### 0.2 Held back on purpose (exists locally, not shipped)
- `src/jetbot.cpp` 200 Hz loop-rate patch: not built, not committed, not deployed.
- `CLAUDE.md` (the assistant's project instructions): kept out of the repository.
- The Jetson's own experimental Git edits (staged and unstaged): never committed, never reconciled with the repo.

### 0.3 Future implementation plans (none of these is built)

Near term, to finish the in-progress work:
1. Install the manager on the robot (`sudo bash ~/jetbot_manager/install.sh`) so the firewall and manager survive reboots.
2. Run the Robot tab live, with the robot present: start navigation, map a new area, save, switch maps, delete and restore. Fix whatever fails.
3. Re-map the room, including the far furnished end, and test whether it is good enough for navigation.
4. Run the lowered AMCL noise in motion and see whether the long-run drift improves. Investigate the 0.05 to 0.16 m y offset between the scan fit and AMCL.
5. Move the "home" goal out of the tight start corner (for example to 0.5, 0.4).
6. Check whether the odometry correction survives a power cycle.

Medium term, new features:
7. **Real emergency stop** using `twist_mux` on the robot so that teleop, `move_base` and a stop command have defined priorities (needs sudo; a design and launch file can be prepared first).
8. **"Find robot" discovery** in the GUI for demos on new networks (`ubuntu.local` did not resolve from the Mac).
9. **Map editor** in the GUI: erase stray cells and draw virtual walls, always saving a new map (never overwriting). It would have prevented the false-obstacle problem at the start corner.
10. A udev rule so the chassis and LiDAR serial ports never swap.
11. Camera and battery telemetry in the GUI (only if the hardware exposes it).
12. Reconcile the Jetson's Git state with the repository, and decide whether to build and test the 200 Hz loop-rate patch.

Possible, not committed to: porting the console to other platforms, and any ROS 2 migration (the physical robot runs ROS 1 Melodic, so nothing here depends on ROS 2).

### 0.4 One-line summary for the documents
A calibrated JetBot Pro that maps a room, localises with AMCL and drives to clicked points under supervision, controlled from a Mac GUI that is finished and
tested; the whole-robot manager (start, map, save and switch maps from the GUI) is built and tested against fakes but still has to be installed and run on the live robot.

---

## 1. System at a glance

| Item | Value |
|---|---|
| Robot | Waveshare JetBot Pro chassis (differential drive) with NVIDIA Jetson Nano 4 GB |
| Robot OS / middleware | Ubuntu 18.04, ROS 1 Melodic. Python 3.6.9 on the robot (matters: no `from __future__ import annotations`) |
| Workspace / package | `/home/nvidia/catkin_ws`, package `jetbot_pro` (a fork of waveshare/jetbot_pro) |
| LiDAR | RPLidar, `/dev/ttyACM1` (previously seen), 115200 baud, frame `laser_frame`, about 7 Hz, range 0.15 to 12 m |
| Chassis serial | `/dev/ttyACM0`, 115200 baud (a micro-controller does the motor control and computes odometry) |
| Camera | CSI camera (started by `slam.launch`, not used in the final navigation) |
| Development machine | Apple Silicon Mac (macOS). It has NO ROS. It runs only the GUI and the tests |
| Robot address | was 192.168.68.107 (it changed over time: .108 earlier). User `nvidia`. Always re-check |
| Software stack | `robot_pose_ekf`, `slam_gmapping`, `amcl`, `map_server`, `move_base` (GlobalPlanner + TEB local planner), `rosbridge_suite` 0.11.16 with `rosapi` |

Frames: `map -> odom -> base_footprint -> laser_frame`. Odometry chain: `/jetbot` publishes `/odom_raw`; `robot_pose_ekf` makes `/odom_combined`; `odom_ekf.py` republishes
it as `/odom`.

Components built during this project (all in the repo):
1. Fixes to odometry and the LiDAR transform.
2. Calibration of the odometry (two correction factors).
3. A living-room map and AMCL localisation, plus a tuned `move_base` config for a small robot.
4. A Mac GUI "navigation console": live map, robot pose, LiDAR, click-to-go, saved places, routes, teleop, pose-versus-scan check and relocalisation.
5. A "JetBot manager" service on the robot, and a Robot tab in the GUI, to start/stop the software, map new areas and manage maps (**work in progress, not yet run live**).

---

## 2. Safety rules the project ran under (state these in the manual)

- The assistant never published `/cmd_vel`, sent navigation goals or started movement. The human drove the robot (keyboard teleop) or pasted the goal command.
- In the GUI every motion action sits behind a **MOTION** master switch (off by default, 5 minute idle time-out) and a safety gate. Teleop speed caps are
  0.25 m/s and 0.8 rad/s, enforced twice, with a hold time of 0.7 s, a LiDAR bumper, and a stop on focus loss, Esc or disconnect.
- Maps are never overwritten. "Delete" moves a map to a recoverable trash folder.
- rosbridge has NO authentication and listens on every network interface, even if you pass an `address` argument. It must be firewalled (section 5, step 7) and reached through an SSH tunnel.
- A real hardware emergency stop is NOT built. The proper fix is a `twist_mux` on the robot (needs sudo). Do not describe the GUI STOP as a hardware e-stop.
- Keep walls at least 50 cm and furniture at least 40 cm away while mapping, and stop 10 to 20 cm short of obstacles when driving by hand.

---

## 3. What is where in the repository

| Path | Purpose |
|---|---|
| `launch/jetbot.launch` | chassis driver, `robot_pose_ekf`, `odom_ekf.py`, IMU static transform |
| `launch/lidar.launch` | RPLidar driver and the `base_footprint -> laser_frame` static transform |
| `launch/localize.launch` | chassis + lidar + `map_server` + AMCL. Launch-time overrides for the corrections and AMCL tuning |
| `launch/navigate.launch` | `localize.launch` + `move_base` using `config/living_room/`. Default `cmd_vel_topic` is `cmd_vel_dry` (a dry run that cannot move the robot) |
| `launch/mapping.launch` | chassis + lidar + GMapping, with the map bounds set by an `extent` argument |
| `launch/gmapping.launch`, `slam.launch` | the original SLAM launches (bounds changed to plus or minus 8 m) |
| `launch/rosbridge.launch` | rosbridge websocket + rosapi (with a security warning) |
| `config/living_room/*.yaml` | tuned costmap, global planner and TEB local planner config for this robot |
| `scripts/odom_ekf.py`, `test/test_odom_ekf.py` | the odometry republisher (rewritten) and its test |
| `tools/jetson_readonly_diag.sh`, `scan_dump.py`, `scan_map_fit.py` | read-only diagnostics, scan dump on the robot, and a scan-to-map pose fitter used as ground truth |
| `gui/` | the Mac console (Tk + roslibpy). `gui/README.md`, `gui/robot_side/ROBOT_SETUP.md` |
| `manager/` | the robot-side manager service, systemd units, install script, README, 33 tests |
| `docs/bringup_findings_2026-11-02.md` | the long findings report from the bring-up |
| `src/jetbot.cpp` | the chassis driver. A 200 Hz loop-rate patch exists locally but was deliberately NOT committed or built |

Git: `main` (default), `claude/jetbot-diagnostics` (stable line), `claude/robot-manager-wip` (adds the manager and Robot tab, unfinished). Milestone tags
`jetbot/01…08` mark stages; `jetbot/07-console-stable` is the rollback point before the manager work.

---

## 4. Setup manual source: step by step (A)

Verified items are marked (verified). Items not yet run live are marked (NOT yet verified).

### 4.1 Prepare the robot
1. Flash/boot the Jetson Nano with the JetBot Pro image (Ubuntu 18.04 + ROS Melodic). Connect it to Wi-Fi and find its IP (`hostname -I` on the robot or the router's device list).
2. Set up SSH key login from the Mac so no password is typed: `ssh-copy-id nvidia@<robot-ip>`. The console uses your normal ssh key; nothing is typed into the app.
3. Clone this repo into the catkin workspace: `~/catkin_ws/src/jetbot_pro`, then build: `cd ~/catkin_ws && catkin_make`.
4. Source ROS in every terminal:
   `source /opt/ros/melodic/setup.bash` and `source ~/catkin_ws/devel/setup.bash`.
5. Check the two serial devices (`ls /dev/ttyACM*`). Their order can swap between boots; the chassis is expected on ttyACM0 and the LiDAR on ttyACM1.
   A udev rule would fix the order permanently (not done).
6. Install the rosbridge packages (needs sudo, done by the user): `sudo apt install ros-melodic-rosbridge-suite`.

### 4.2 Fix the LiDAR orientation (important, verified)
The RPLidar's 0 degree direction faces the robot's **rear**. The static transform in `launch/lidar.launch` must therefore use yaw **3.14**:
`0 0 0.15 3.14 0 0 /base_footprint /laser_frame`. Yaw 0 puts the whole map backwards. (The repo's original value was 3.14; an experiment on the Jetson had changed it to 0, which was wrong.)
The driver also prints "laser mounted upwards"; that is the driver's own notice and did not need action.

### 4.3 Odometry: the two correction factors (verified)
The chassis micro-controller computes odometry and runs the motors. At start-up the driver sends it `linear_correction` and `angular_correction`. These also change the real
speed and rotation, so they cannot be fixed in software afterwards. The working values, found by measurement, are:
- `linear_correction` = **1.15**, `angular_correction` = **0.9** (vendor default for both is 1.0).
- At 1.0 the robot under-read distance by about 24 percent. At 1.4 it over-read by about 48 percent.
- They are now the DEFAULTS in `jetbot.launch`, `localize.launch`, `navigate.launch` and `mapping.launch`, so a plain launch is calibrated. Pass `linear_correction:=` / `angular_correction:=` only to recalibrate. (Until 2026-11-22 the defaults were the vendor 1.0 and had to be passed by hand; that was changed so nobody has to remember it. The robot-side copy of the launch files must be redeployed for this to take effect on the Jetson.)
- Unknown: whether the correction persists in the micro-controller after a power cycle. Re-check after any power cycle.

### 4.4 Make a map (verified)
1. Launch mapping (chassis + lidar + GMapping): in the final design this is `mapping.launch` (or the original `slam.launch`).
2. Drive slowly with keyboard teleop. Keys used: `i` forward, `,` backward, `j` left, `l` right, `k` stop. Keep clear of walls and furniture as in section 2.
3. Save the map with `map_saver` to a NEW name under `~/maps` (never overwrite). The living-room map was saved as `livingroom_20261102_0830` and then edited (see 5.5).
4. Do not trust a map because `/map` exists. Check it visually: the first saved map had 8 false occupied cells at the start position, which blocked planning. A cleaned copy (`_start_cleaned`) and a walled copy (`_walled`, with a virtual wall) were made as new files.

### 4.5 Localise and navigate (verified in supervised runs)
1. Dry run first: `roslaunch jetbot_pro navigate.launch` (default `cmd_vel_topic:=cmd_vel_dry`, cannot move the robot).
2. Real run, with a person supervising and an initial pose that matches where the robot really is:
   `roslaunch jetbot_pro navigate.launch cmd_vel_topic:=cmd_vel initial_x:=<m> initial_y:=<m> initial_a:=<rad>`
3. AMCL must be given the right start pose. After the robot is hand-carried, the pose is always wrong. Re-seed it by relaunching with `initial_x/y/a`, or in the GUI with "Check pose from scan" then "Apply to AMCL".
4. Navigation parameters for this small robot (in `config/living_room/`): TEB `max_vel_x` 0.22, `max_vel_theta` 0.6, backwards speed 0.15, acceleration 0.3 and 0.6, `min_obstacle_dist` 0.10, goal tolerance 0.15 m and 0.25 rad, circular footprint of radius 0.12 m,
   unknown space not allowed, rotate recovery off, controller frequency 6 Hz.
5. Tuned AMCL: `z_hit` 0.9, `z_rand` 0.1, motion noise alpha 0.12 / 0.10 / 0.20 / 0.10, `sigma_hit` 0.10. These are deployed but were NOT yet run in motion (NOT yet verified).

### 4.6 The Mac GUI (verified, the user confirmed it works)
1. On the Mac: create a virtual environment in `gui/`, `pip install -r requirements.txt` (and `requirements-dev.txt` for the tests). It needs Python with Tk and `roslibpy`.
2. Start rosbridge on the robot (`launch/rosbridge.launch`) AFTER the firewall rule (4.7).
3. Open an SSH tunnel: `ssh -N -L 9090:127.0.0.1:9090 nvidia@<robot-ip>`, then run the GUI and connect to host `127.0.0.1`. The final design opens this tunnel itself from the Connect button.
4. Use: switch MOTION on, click a point on the map, press GO (or double-click). Shift+drag sets the heading. Space or STOP cancels. Tabs: Robot, Navigate, Teleop, Pose, Diagnostics.
5. Teleop tab: hold buttons or use `i , j l k`. Saved places: name a spot, then go to it by name. Routes: queue several goals; the runner stops at the first failure.
6. Tests (no robot, no ROS): `python -I -m unittest discover -s tests` from `gui/`. 146 tests; they passed 15 times in a row after the flaky-test fix.

### 4.7 Firewall for rosbridge (verified, must be redone after each reboot unless the manager's systemd unit is installed)
rosbridge listens on 0.0.0.0 with no authentication. Block outside access to port 9090 before starting it:
`sudo iptables -I INPUT -p tcp -m tcp --dport 9090 ! -i lo -j DROP` (and the `ip6tables` equivalent). Reach it only through the SSH tunnel. This rule was verified with a dummy listener and `nc`.

### 4.8 The robot manager (WORK IN PROGRESS, NOT yet run end to end on the live robot)
Purpose: start navigation or mapping from the GUI, save/rename/delete/restore maps, and keep rosbridge up. It is a stdlib-only HTTP service on 127.0.0.1:9091 with a secret token,
a fixed action set, validated map names, and a recoverable trash. Install once with sudo: `DRY_RUN=1 bash ~/jetbot_manager/install.sh` to preview, then `sudo bash ~/jetbot_manager/install.sh`.
It installs `jetbot-firewall.service` and `jetbot-manager.service`. Full details are in `manager/README.md`. The read-only endpoints, rename, delete and restore were tested on the robot on a spare port with a throwaway map; the systemd install and
start-navigation / map-new-area flows were never run on the live robot.

---

## 5. Project report source: what was found and what was fixed (B)

### 5.1 Major problems, in the order met, each with cause and fix

| # | Symptom | Real cause (as established) | Fix / outcome |
|---|---|---|---|
| 1 | The map looked mirrored or backwards; scans did not match the room | RPLidar 0 deg faces the rear; the Jetson copy had its static transform yaw changed from 3.14 to 0 as an experiment | Restored yaw 3.14 (approved by the user and deployed) |
| 2 | `/odom` looked wrong: huge covariances (about 1.5e10), no useful velocity, hidden failures | `odom_ekf.py` had a bare `except: pass` hiding errors, used `/odom` as the frame id (leading slash), blocked in its constructor, copied pose covariance only and left twist empty | Rewrote `scripts/odom_ekf.py` with proper error handling, frame ids and twist, and added `test/test_odom_ekf.py` (deployed after approval) |
| 3 | Robot moved a little and stopped; costmaps looked empty; goals failed | Mixed: odometry quality, then a map with false obstacles and too-aggressive parameters for a 20 cm robot (0.7 m/s, 2.5 rad/s, 4 Hz controller) | A new conservative `config/living_room/` set (speeds, footprint 0.12 m, inflation, `min_obstacle_dist` 0.10, 6 Hz controller) |
| 4 | Driving "1 m" went 0.76 m, then after a first correction attempt went too far | Calibration was first based on biased references (a tape measure and AMCL) and gave `linear_correction` 1.4, which over-read by 48 percent | Built `tools/scan_map_fit.py`: slide the live scan over the saved map and maximise the fraction of points within 5 cm of a wall. Using it as ground truth gave **1.15 and 0.9** |
| 5 | AMCL pose drifted 0.35 m in x and 30 degrees in heading on long moves; the GUI said "lidar mislocalised" | AMCL drifts on long runs and is always wrong after a hand-carry. The exact cause (motion noise or the far, furnished end of the map) is unproven | Re-seed by relaunch or by the GUI relocalise buttons; lowered AMCL motion noise (untested in motion). Also fixed one case where I re-seeded at the wrong place because the robot had already been carried back |
| 6 | Planner abort "trajectory not feasible" at the start | The saved map had 8 false occupied cells at the start position | Made a cleaned copy of the map (new file) |
| 7 | Return goal "no plan" | AMCL's x error put the rear wall on top of the goal | Re-seed plus the new AMCL weights |
| 8 | Long route: legs 1 to 3 reached, leg 4 aborted | The goal was in the tight start corner | Use a home goal away from the corner (for example 0.5, 0.4) |
| 9 | rosbridge was exposed on all interfaces for about 30 s | It ignores the `address` setting and listens on 0.0.0.0 with no authentication | Stopped it at once, added the iptables rule, verified it, corrected the launch file warning and the docs |
| 10 | The user wanted click-to-go, not drag | First UI used drag | Redesigned: click, GO or double-click, Shift+drag for heading |
| 11 | GUI clipped status cards and was hard to read | Fixed-size layout | Numeric layout audit, wrapped cards, scrollable tabs (the screen could not be captured by the assistant, so layout was checked numerically) |
| 12 | The manager would not run on the robot | Used `from __future__ import annotations` and other Python 3.7+ features; the Jetson has Python 3.6.9 | Removed them, tested on the robot's Python |
| 13 | Robot Tab buttons wrong at start | Button states were computed before the default map was selected | Fill the map list first |
| 14 | One GUI test failed about 3 times in 10 full runs | UI tests destroyed windows without stopping the window's rosbridge retry loop. Tk variables were later freed inside roslibpy's reactor thread, where that blocks, so a later test could not connect (found with a thread stack dump) | Tests now stop the links and collect garbage on the main thread; the fake server uses a free port per test. 15 consecutive full passes |

### 5.2 Smaller mistakes and tooling problems worth a paragraph
- Two edits to `rosbridge.launch` produced invalid XML: the comment contained a double hyphen (`--dport`). Lesson: never put two hyphens in an XML comment, and parse launch files before deploying.
- A remote `pkill -f` / `pgrep -f` pattern matched its own shell and killed the SSH session several times. Fix: anchored patterns, for example `^/usr/bin/python /opt/ros/melodic/bin/roslaunch ...`.
- A test left a fake server running and caused a port clash; the test now stops it.
- macOS differences: `sed -i` syntax and no `getent`. Fixed with a Python patch and a portable fallback in `install.sh`.
- `sudo` needs a password. The assistant never guessed it; the user ran the apt install and the firewall command themselves. The manager's systemd install is still pending for the same reason.
- The Jetson rebooted at least once, which lost the firewall rule and stopped ROS. This is why the firewall needs a systemd unit.
- Git: the work was first committed in 18 coarse commits. At the user's request it was rebuilt as 59 small commits (stable line) plus 13 (manager and WIP), with the file contents proven identical at every original boundary (tree hash `4cff7c10…`),
  plus 8 milestone tags. The old history is kept only as a local tag `backup/before-history-split`.
- GitHub: the user's `yeses03/jetbot_pro` is a fork, and commits on forks do not show in the contribution graph, so everything was also pushed to a new non-fork repo, `yeses03/jetbotpro`.
  The contribution graph also needs the commit email (`lyeseswini@gmail.com`) to be added to the GitHub account.

### 5.3 Key technical decisions and why
- Odometry is computed in the chassis firmware (not in this repo), so calibration had to use the driver's start-up correction parameters rather than a software fix.
- Ground truth for calibration came from the scan-to-map fit, because tape measure and AMCL were both biased.
- Navigation uses `move_base` (ROS 1) with the GlobalPlanner and TEB, not Nav2. A friend's reference GUI was ROS 2 and was NOT ported directly.
- The GUI talks to the robot only through rosbridge (websocket) over an SSH tunnel, and publishes only to the goal, cancel, `/initialpose` and (teleop only, capped) `/cmd_vel` topics.
- The manager is kept separate from ROS so the robot software can be started and stopped from outside ROS; it is localhost-only, token-protected and cannot move the robot.
- Default launch is a dry run (`cmd_vel_dry`), so a mistaken launch cannot move the robot.

### 5.4 Results that were actually achieved (verified)
- Odometry calibrated; distance and rotation errors went from tens of percent to small, using the scan-fit ground truth.
- The living-room map was built and saved; AMCL localised on it; supervised autonomous goals were reached (including a longer route, legs 1 to 3).
- The GUI works (the user confirmed it): click-to-go with a safety gate, saved places, routes, relocalisation and teleop.
- 146 automated GUI tests and 33 manager tests pass without a robot.

### 5.5 Not proven or not done (state these honestly in the report)
- AMCL's long-run drift cause is unproven; the lowered motion noise is untested in motion.
- The scan-fit y is 0.05 to 0.16 m above AMCL's y in every run; unexplained.
- Whether the far, furnished end of the room is mapped well enough for navigation is untested (a re-map is needed).
- Whether the odometry correction persists across a power cycle is unknown.
- The Robot tab, SSH tunnel flow and manager have never been run end to end on the live robot; the manager's systemd install has not been run.
- No real emergency stop (`twist_mux` needed).
- The original symptoms of "stops when turned in one direction" were not individually root-caused beyond the points above.
- The held `src/jetbot.cpp` 200 Hz loop-rate patch is unbuilt and uncommitted; the Jetson's own Git state (staged experimental edits) was never reconciled.
- Camera and battery telemetry are not implemented.
- A discovery ("find robot") feature and a map editor (virtual walls, erase) are ideas only.

---

## 6. Timeline for the report
- **2026-11-01**: read-only audit; found the lidar yaw and odometry problems; odometry fix approved, deployed, committed.
- **2026-11-02 morning**: living-room mapping (photos used to plan), map saved, AMCL set up.
- **2026-11-02 midday**: odometry calibration (first 1.4, then corrected to 1.15 / 0.9 using scan-fit ground truth); first supervised goals.
- **2026-11-02 afternoon**: rosbridge installed and firewalled; GUI built; long-route test; relocalisation and saved places; click-to-go redesign; teleop; UI polish.
- **2026-11-02 evening**: manager and Robot tab started; history rebuilt as 72 small commits with milestone tags.
- **2026-11-04**: flaky-test fix, manager README, GUI docs, push to `main`.
- **Later**: pushed everything to the new non-fork repo `yeses03/jetbotpro` so commits show on the user's contribution graph.

---

## 7. Hardware and numbers cheat sheet
- Chassis width about 19.5 to 20 cm; planner footprint circular, radius 0.12 m.
- LiDAR mounted 0.15 m above `base_footprint`; IMU static transform z = 0.07 m.
- Map resolution 0.05 m per cell; GMapping bounds plus or minus 8 m (320 by 320 cells).
- Teleop caps 0.25 m/s and 0.8 rad/s; TEB planning caps 0.22 m/s and 0.6 rad/s.
- Working corrections: linear 1.15, angular 0.9.
- Ports: rosbridge 9090, manager 9091 (both only through the SSH tunnel), ROS master 11311.

---

## 8. Open questions the document writer should flag as "to confirm with the author"
- The exact Jetson image version and the physical wheel radius / track width (only the chassis width was estimated).
- The final robot IP (it changed) and whether a udev rule for the serial ports will be added.
- Photos or measurements of the physical room and the sensor mounting, which the report may want but this material does not contain.

## 9. Companion files in the repo (give these to the doc assistant too)
- `docs/bringup_findings_2026-11-02.md` (the long findings and replication checklist)
- `gui/README.md` and `gui/robot_side/ROBOT_SETUP.md`
- `manager/README.md`
- `launch/navigate.launch`, `launch/localize.launch`, `launch/mapping.launch`, `launch/rosbridge.launch`
- `config/living_room/*.yaml`
- `git log --oneline` on `main`, and the tags `jetbot/01…08` for the build order
