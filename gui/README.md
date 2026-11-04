# JetBot Pro navigation console (map, LiDAR, pose, click-to-go, teleop)

A desktop GUI (Python + Tkinter) that shows what the JetBot is actually doing: map, LiDAR, robot pose, link health.
It runs on your Mac/Ubuntu VM and talks to the **ROS 1 Melodic** robot over the network through **rosbridge**.
No ROS is needed on the GUI machine, and nothing on the robot is migrated to ROS 2.

**What it can and cannot do to the robot.** It can (1) send **goals** to `move_base` and cancel them, (2) **teleoperate** the robot with the same keys as
`teleop_twist_keyboard`, and (3) tell AMCL where the robot is. Nothing else: the link refuses every other topic, and the only velocity path is the
teleop pad, which is hard-capped (0.25 m/s, 0.8 rad/s), guarded by the LiDAR, and times out by itself. Everything that can move the robot sits behind
one **MOTION** master switch (off by default, and it turns itself off after 5 minutes without a command). There is no emergency stop: **STOP**
cancels goals and teleop, but the **power switch is the real stop**.

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
pip install -r requirements.txt            # roslibpy, numpy, pillow  (requirements-dev.txt adds websockets for the tests)
# Ubuntu also needs Tk:  sudo apt install python3-tk
ssh -N -L 9090:127.0.0.1:9090 nvidia@192.168.68.107 &     # tunnel (rosbridge listens on localhost only)
python -m jetbot_gui --host 127.0.0.1
```
(`python3-tk` is a system package. On Homebrew Python for macOS use `brew install python-tk`.)

## Using the console (it moves the robot: read this first)

Layout: a header with the connection, the **MOTION** switch and **STOP**; status cards; the map; and a side panel with tabs: *Robot*, *Navigate*,
*Teleop*, *Pose*, *Diagnostics*.

**Click-to-go.**
1. Switch **MOTION** on (top right). Nothing can move the robot while it is off.
2. **Click the map** where the robot should go. An orange marker and a bar with **GO** appear. A plain click faces the robot along its direction of travel.
   **Shift + drag** sets the heading instead. **Drag** pans, the **wheel** zooms. **Double-click** chooses and goes in one gesture.
3. Press **GO**. The robot drives there with `move_base`, avoiding obstacles. **STOP** (or the Space key) cancels it.

**The safety gate.** GO is blocked, with the reason shown, unless: MOTION is on; the link is up; the map pose is live; `move_base` is running; the robot's
pose agrees with the LiDAR scan (*Pose vs scan* not red); AMCL's uncertainty is small; no goal is already active; and teleop is off. The gate checks
localisation, not the world: it does not know if a person or pet is in the way.

**On the map.** The robot is drawn with its planner footprint (dashed circle, 0.12 m radius), the LiDAR scan in red, your destination in orange, the goal being driven to in blue with the planned path as a blue line, route legs in green, saved places as purple diamonds, and a 1 m scale bar. The path line is read-only and only shown while a goal is active.

**Routes.** *+ Route* queues the destination (green numbered markers). *Run route* (Navigate tab) sends the legs one at a time and only moves on after
`move_base` reports the previous goal reached. The first failure, cancellation or failed gate check stops it; it never retries.

**Saved places.** Give a spot a name once, then go there by name: choose a destination (or put the robot there) and use *Save current as*; later pick it
in the dropdown and press *Go to place* (same gate as any goal), *+ Route*, or *Delete*. Places show as purple diamonds. They are stored in
`places.local.json` (git-ignored) per map, together with the shape of the map they were made on, and are refused on a different map.

## Robot tab (start the robot software, map a new area, manage maps)

*Work in progress: tested against fakes and a throwaway map on the robot, not yet run end to end on the live robot.*

This needs the **JetBot manager** running on the Jetson (`manager/README.md`; one-time `sudo` install). Enter the robot's address and user in the
header and press **Connect**: the console opens an SSH tunnel itself (using your normal ssh key, nothing is typed into the app), fetches the manager's
token over it, and talks to rosbridge and the manager through two local ports. Nothing is exposed on the network.

- **Start navigation on selected map** starts the chassis, LiDAR, AMCL and `move_base` on the map chosen in the library. It does not move the robot;
  motion still needs the MOTION switch.
- **Map a new area** starts the chassis, LiDAR and GMapping. Drive around with the Teleop tab, then name the map and press **Save map**. An
  existing name is refused, so no map is ever overwritten.
- **Stop software** stops whichever of the two is running.
- **Map library**: previews, **Rename**, **Delete** (moves the map to a recoverable trash list, **Restore** brings it back), **Refresh**.
- **Places are per map.** Switching maps shows that map's places only; places saved before this existed are in the "default" set.

## Teleop (drive with the keyboard)

Switch **MOTION** on, open the **Teleop** tab, press **Start teleop** and confirm. Keys, as in `teleop_twist_keyboard`:

| Key | Action |
|---|---|
| `I` | forward |
| `,` | backward |
| `J` | turn left |
| `L` | turn right |
| `K` or Space | stop |
| Esc | teleop off |

The on-screen pad does the same while held. Diagonal keys (`U`, `O`, `M`, `.`) and speed keys (`Q`, `W`, `X`, `Z`) are deliberately **not** mapped.
Speeds come from two sliders and are hard-capped.

What keeps it safe, and what does not:
* A key press holds the command for **0.7 s** (the same as the `_key_timeout` you have been using), so tap or hold; a lost key-release, a crash or a lost
  window focus stops the robot within that time (and the driver's own 1 s command watchdog backs that up). Losing window focus, Esc, switching MOTION
  off, disconnecting, or closing the window all stop teleop and send zero commands.
* Starting teleop cancels any active goal, and goals are blocked while teleop is on, so `move_base` and the pad never fight over `/cmd_vel`.
* A **LiDAR bumper** blocks motion towards something close: forward and reverse if a point is within 0.42 m in the robot's 0.4 m wide corridor, turning if
  anything is within 0.23 m. It refuses to move at all without a live scan. It is a convenience, not a safety system: the LiDAR cannot see closer than
  about 0.15 m, glass, or anything above or below its plane. Reverse is also blocked by a charger cable behind the robot.
* Typing in a text box (for example a place name) never drives the robot.
* There is no emergency stop in software. Keep a hand near the power switch.

## Fixing a wrong pose: "Check pose from scan"

AMCL can drift on long moves, and it is always wrong after you lift or push the robot (odometry cannot see that). The *Pose vs scan*
chip turns red when that happens and the goal buttons are blocked. To fix it from the dashboard:
1. Put the robot where it is going to stay, still, with nobody next to it (a person in the scan lowers the fit).
2. Press **Check pose from scan**. It searches for the pose where the LiDAR scan fits the saved map best, near the reported pose, or
   across the **whole room** if you tick *Search the whole room* (use that after carrying the robot). It takes a second or two.
3. Read the result: the suggested pose (cyan cross on the map) and its fit. It is only offered when the fit is good (>= 70%) and
   no other place fits almost as well; a plain rectangular room can look the same turned 180 degrees, and the panel says AMBIGUOUS then.
4. If the cross is where the robot really is, press **Apply to AMCL** and confirm. This publishes AMCL's initial pose; it does not move
   the robot, but every goal afterwards is planned from it. Check that *Pose vs scan* turns green.

The link may publish to exactly four topics: the goal, goal cancel, the AMCL initial pose, and (teleop only, capped) the velocity topic.

## Configuration

Order: built-in defaults < `config.default.json` < `config.local.json` (git-ignored) < command-line flags.
The **Save as default** button writes `config.local.json`. Every topic and frame name is a setting
(`--map-topic`, `--scan-topic`, `--odom-topic`, `--amcl-pose-topic`, `--map-frame`, `--odom-frame`, `--base-frame`).
Defaults come from this repo's launch files: `/map`, `/scan`, `/odom`, `/amcl_pose`, frames `map`, `odom`, `base_footprint`.
The default host is `127.0.0.1`: rosbridge is started by `launch/rosbridge.launch`; **it listens on all interfaces whatever `address` says, so block port 9090 from outside with a firewall rule first (robot_side/ROBOT_SETUP.md section 3)**, then reach it through an SSH tunnel to the Jetson (its address changes, it was .108 and then .107). Change the host in the toolbar or `config.local.json` if you connect directly on a trusted network.

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
`test_teleop.py` checks the key map, the 0.7 s hold, the speed caps (also inside the link), the LiDAR bumper, and that the velocity message is clamped and goes to the velocity topic only. `test_ui_smoke.py` drives the real window: click vs drag vs Shift+drag vs double-click, the MOTION switch and its timeout, STOP, and teleop (keys, pad, focus loss, typing in a text box). `test_goals.py` checks the goal message, the safety gate, the route runner (sends the next leg only after success, stops at the first failure), that the link refuses every topic except goal/cancel, that no velocity topic appears anywhere in the package, and that a goal and a cancel reach a fake rosbridge server correctly. `test_reloc.py` checks the scan-to-map pose search on real scans (it recovers the 0.35 m and 30 degree errors AMCL made, with and without a guess, and reports ambiguity) and the initial-pose message; the UI tests run the check-then-apply flow. `test_places.py` checks saving, name validation, atomic writes, the different-map refusal, and the go-to-place flow with a declined/accepted confirmation. `test_fit.py` checks the pose-vs-scan indicator on a synthetic room and on **real scans captured on the robot** (`tests/fixtures/`: the cleaned living-room map and three scans, with the pose AMCL reported and the pose a scan-to-map search found). `test_ui_smoke.py` builds the real Tk window, feeds it that real data and renders one refresh; it skips when there is no display.
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

## Roadmap

Done: dashboard, pose-vs-scan check, click-to-go, routes, saved places, relocalisation, teleop with a LiDAR bumper, tabbed UI. Not built: a real emergency
stop (it needs a velocity multiplexer such as `twist_mux` on the robot, so that teleop, `move_base` and a stop command have defined priorities), camera and
battery telemetry.
