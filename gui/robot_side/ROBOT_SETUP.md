# Robot-side setup for the dashboard (run on the Jetson only after you approve)

The GUI needs exactly one thing the robot may not have yet: **rosbridge**. Nothing else on the robot is changed,
and no existing launch file or parameter is edited.

## 1. Install (Jetson, ROS Melodic, Ubuntu 18.04)
```bash
sudo apt update
sudo apt install ros-melodic-rosbridge-suite      # includes rosbridge_server and rosapi
```
Check first whether it is already there: `rospack find rosbridge_server`.
On the project's Jetson (checked 2026-11-02) it is **not installed**: `ros-melodic-rosbridge-server` 0.11.16 is available from the ROS apt repo, and `sudo` asks for a password, so install it from a terminal where you can type it. Nothing was installed by the assistant.

## 2. Run (in a separate terminal, alongside your existing launch, e.g. nav.launch or slam.launch)
```bash
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash
roslaunch jetbot_pro rosbridge.launch                      # wraps rosbridge_websocket (listens on ALL interfaces, see section 3), also starts rosapi
# from your computer:  ssh -N -L 9090:127.0.0.1:9090 nvidia@<jetson-ip>   and point the GUI at host 127.0.0.1
```
Do not start a second roscore/jetbot/lidar/SLAM; rosbridge just joins the running ROS master.

## 3. Security note (read before starting rosbridge)
**Verified 2026-11-02 on the project's Jetson: with `rosbridge_server` 0.11.16 the `address` launch argument does NOT restrict the
listening interface.** It listened on `0.0.0.0:9090` even with `address:=127.0.0.1`, so the robot's whole network could publish to
`/cmd_vel`. It was up for about 30 s before being stopped. Block outside access first, on the Jetson:
```bash
sudo iptables -I INPUT -p tcp --dport 9090 ! -i lo -j DROP     # not persistent across reboots
```
then check from another machine that `nc -vz <jetson-ip> 9090` fails, and only then start rosbridge and use the SSH tunnel.

rosbridge has **no authentication**. Anyone on the same network can publish to any topic, including `/cmd_vel`.
Stage 2 of the GUI only reads, but the server itself is open. Use a trusted network, stop rosbridge when not needed, and
decide on firewalling (`ufw allow from <your-ip> to any port 9090`) before the control stage.

## 4. Verify what the GUI assumes (on the Jetson, with the stack running)
```bash
rostopic list                       # /map /scan /odom /tf /tf_static present?
rostopic info /scan                 # type sensor_msgs/LaserScan; who publishes?
rostopic echo -n1 /scan/header      # real frame_id (repo's lidar.launch uses laser_frame)
rostopic hz /scan /tf
rosrun tf view_frames               # is there map -> odom -> base_footprint -> laser_frame ?
rosnode list                        # amcl OR slam_gmapping, move_base, rplidarNode, jetbot ...
rostopic info /map                  # latched? published by map_server (nav) or slam_gmapping?
```

## 5. Findings from the 2026-11-02 bring-up that the dashboard is meant to expose
* **Odometry is not calibrated yet** (the firmware scale error is about -24% at the default setting, and the correction knob
  also changes real speed and rotation), and **AMCL can be 0.35 m or 30 degrees wrong while still publishing a confident-looking
  pose**. That is why the *Pose vs scan* chip exists: it compares the pose with the live scan against the saved map.
* The planner and AMCL both failed safely when the pose was wrong; the dashboard should make that visible before a goal is sent.

## 6. Findings from the repo that affect later stages (not verified on the physical robot)
* **No velocity multiplexer.** `src/jetbot.cpp` subscribes to `/cmd_vel` directly, and `move_base` (TEB planner) publishes
  to `/cmd_vel` too. A GUI teleop publishing there would fight `move_base`. Manual mode must first cancel the goal; a real
  fix is a `twist_mux` on the robot.
* **The driver has a 1 s command watchdog** (it zeroes velocity if no `/cmd_vel` arrives for 1 s), which limits a runaway
  after a lost connection. This is a software timeout on the Jetson, not a hardware safety stop.
* **A software e-stop cannot be guaranteed from the GUI alone**: it can cancel `move_base` goals and stream zeros, but
  anything else still publishing `/cmd_vel` (e.g. `laser_avoidance.py`) can override it, and a dead network link means no
  command arrives at all (the 1 s watchdog is then the only protection).
* **LiDAR TF**: this repo's `lidar.launch` publishes `base_footprint -> laser_frame` with yaw `3.14`; you mentioned a past
  experiment changed it to `0.0`. Check which is live on the robot and which matches the real sensor mounting.
* **IMU**: the repo's `jetbot.launch` has `imu_used=true` in `robot_pose_ekf`; you mentioned disabling it. Your Jetson's
  working copy may differ from the repo.
* **Odometry path**: `jetbot` publishes `/odom_raw`; `robot_pose_ekf` → `/odom_combined`; `odom_ekf.py` republishes it as `/odom`.
* **No battery topic** appears in `src/jetbot.cpp`; **camera** is `gscam` raw `/csi_cam_0/image_raw` (too heavy as JSON; would
  need a compressed relay). Both stay "not configured" until you identify real sources.
* **No saved map in the repo** (`maps/` holds only scripts); `nav.launch` defaults to `maps/mymap.yaml`, so pass `map_file:=…`.
