# JetBot Pro dashboard (Stage 2: read-only)

A desktop GUI (Python + Tkinter) that shows what the JetBot is actually doing: map, LiDAR, robot pose, link health.
It runs on your Mac/Ubuntu VM and talks to the **ROS 1 Melodic** robot over the network through **rosbridge**.
No ROS is needed on the GUI machine, and nothing on the robot is migrated to ROS 2.

**This build cannot move the robot.** It only subscribes and queries; it never publishes. Manual drive, goals,
waypoints and e-stop are later stages (see "Roadmap") and wait until the command path is verified on the robot.

Adapted (not copied) from the Acrux GUI; none of its ROS 2 / Nav2 / SSH / credential code is used.

## Architecture

```
 Mac / Ubuntu VM                                  Jetson Nano (ROS 1 Melodic, unchanged)
 ┌───────────────────────────┐   websocket :9090  ┌──────────────────────────────────┐
 │ Tkinter UI (app.py)       │   JSON (rosbridge) │ rosbridge_websocket + rosapi     │
 │  └ status.py  (honesty)   │ <────────────────> │  ↕ ROS topics: /map /scan /tf …  │
 │  └ link.py (roslibpy,     │                    │ jetbot, rplidar, amcl/gmapping,  │
 │     background threads)   │                    │ move_base …                      │
 │  └ geometry.py (pure math)│                    └──────────────────────────────────┘
 └───────────────────────────┘
```

Why rosbridge + roslibpy: it is pure Python on the GUI side (works on any OS/Python), needs only one extra apt
package on the Jetson, and leaves the existing launch files and drivers untouched. Alternatives rejected:
installing Melodic in a VM (unsupported on new Ubuntu), a ROS 1 Docker/RoboStack setup on the Mac (heavy, fragile ROS
networking through NAT), SSH polling (cannot stream a map or scan).

Files: `config.py` (settings), `link.py` (network + latest-value store, never blocks the UI), `status.py` (turns raw data
into honest OK/WARN/BAD indicators), `geometry.py` (map rendering, TF tree, scan maths; unit-tested), `app.py` (UI).

## Setup

**Robot (once, needs your approval, nothing here is run automatically):** see [robot_side/ROBOT_SETUP.md](robot_side/ROBOT_SETUP.md).

**GUI machine:**
```bash
cd gui
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt            # roslibpy, numpy, pillow
# Ubuntu also needs Tk:  sudo apt install python3-tk
ssh -N -L 9090:127.0.0.1:9090 nvidia@192.168.68.107 &     # tunnel (rosbridge listens on localhost only)
python -m jetbot_gui --host 127.0.0.1
```
(`python3-tk` is a system package. On Homebrew Python for macOS use `brew install python-tk`.)

## Configuration

Order: built-in defaults < `config.default.json` < `config.local.json` (git-ignored) < command-line flags.
The **Save as default** button writes `config.local.json`. Every topic and frame name is a setting
(`--map-topic`, `--scan-topic`, `--odom-topic`, `--amcl-pose-topic`, `--map-frame`, `--odom-frame`, `--base-frame`).
Defaults come from this repo's launch files: `/map`, `/scan`, `/odom`, `/amcl_pose`, frames `map`, `odom`, `base_footprint`.
The default host is `127.0.0.1`: rosbridge is started by `launch/rosbridge.launch` bound to localhost, and you reach it through an SSH tunnel to the Jetson (its address changes, it was .108 and then .107). Change the host in the toolbar or `config.local.json` if you connect directly on a trusted network.

## What the indicators mean

| Indicator | Meaning |
|---|---|
| Link | websocket state; amber "connected, but no data" if the socket is up but nothing arrives |
| Map | occupancy grid received (size, resolution, age). Latched topics (AMCL's map_server) never update, so age is information, not an error |
| LiDAR | green only if a scan arrived within `scan_stale_s` (2 s). Stale/missing scans are never drawn |
| Pose | **green** = live `map → base_footprint` from TF. **amber "ODOMETRY ONLY"** = no `map → odom` transform, so the robot is *not localised*; it is shown as numbers only and is **not drawn on the map** |
| Odometry | `/odom` freshness |
| Pose vs scan | **independent check of the reported pose**: the share of LiDAR points (drawn through the map pose) that land on mapped walls. Green >= 70%, amber 60-70%, red < 60% (`fit_ok` / `fit_warn` in the config). On the living-room map the true pose scores 0.86-0.87; the pose AMCL reported when it was 0.35 m off scored 0.57 and when its heading was 30 degrees off, 0.36. Red means the pose is probably wrong (or the robot is outside the mapped area); it does **not** say which way. AMCL sigma above 0.3 m / 20 degrees adds a warning |
| Localiser | which of AMCL / GMapping / Hector / Karto / Cartographer is running (via `rosapi`); warns if two run together |
| Navigation | whether `move_base` is running |

Staleness is judged by *local receive time*, never by message timestamps (the Jetson and your machine have different clocks).
The pose comes from composing `/tf` + `/tf_static` inside the GUI, so `tf2_web_republisher` is not needed.
The LiDAR is drawn through whatever transform the robot publishes for its scan frame, so a wrong mounting yaw in
`lidar.launch` shows up as a scan that does not line up with the map. The GUI does not "correct" it.

Map controls: wheel = zoom, drag = pan, **Fit**, **Centre on robot**. The cursor's map coordinates are shown top right.

## Tests (no robot, no ROS)

```bash
cd gui
python -I -m unittest discover -s tests -v
```
`test_fit.py` checks the pose-vs-scan indicator on a synthetic room and on **real scans captured on the robot** (`tests/fixtures/`: the cleaned living-room map and three scans, with the pose AMCL reported and the pose a scan-to-map search found). `test_ui_smoke.py` builds the real Tk window, feeds it that real data and renders one refresh; it skips when there is no display.
`test_core.py` covers map orientation/origin/rotation, scan invalid-value handling, TF composition, staleness and the
status rules. `test_integration.py` runs the real roslibpy client against a **fake rosbridge server that exists only inside
the test** (connect-before-server-exists, live data, server dies → data goes stale, server returns → recovers). The GUI itself
never fabricates data. Requires `pip install websockets` for that one test.

## Troubleshooting

* **"cannot reach ws://…:9090"** – rosbridge is not running on the robot, wrong IP, or a firewall/VM network mode problem
  (use a bridged VM adapter so the VM is on the robot's LAN). Test: `nc -vz <ip> 9090`.
* **Connected, but "no data"** – rosbridge is up but the ROS master/drivers are not; check on the Jetson with `rostopic list`.
* **No map** – nothing publishes `/map` (start `slam.launch` or `nav.launch`), or it has another name (`--map-topic`).
* **Pose says ODOMETRY ONLY** – no AMCL/GMapping is publishing `map → odom`. For AMCL the robot also needs an initial pose.
  (Setting an initial pose from the GUI is a later stage; use RViz for now.)
* **Scan is live but not drawn on the map** – the TF chain `map → <scan frame>` is incomplete or stale; the warning line
  names it. The right-hand LiDAR panel still shows the scan in the robot frame.
* **Large maps are slow** – rosbridge sends the map as JSON; very large grids are heavy. Not yet tested on the real robot.
* **Localiser: "unknown"** – `rosapi` isn't reachable (it is started by `rosbridge_websocket.launch`).

## Roadmap (not implemented)

Stage 3 (manual drive, `move_base` goals/cancel, e-stop) and Stage 4 (waypoints, home, camera, battery) are intentionally
absent; the facts that gate them are in ROBOT_SETUP.md ("To verify before Stage 3").
