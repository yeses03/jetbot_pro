# JetBot manager

A small service that runs on the Jetson and lets the console (the Robot tab) start and stop the robot software and manage maps.
ROS cannot start itself, so something outside ROS has to: this is that something.

It does three jobs:

1. Keeps **rosbridge** (and with it the ROS master) up, so the console can always connect.
2. Starts **one stack at a time**:
   - *navigation*: chassis + lidar + AMCL + `move_base` on a chosen map (`launch/navigate.launch`)
   - *mapping*: chassis + lidar + GMapping (`launch/mapping.launch`), with the map bounds set from the `extent` you choose
3. Manages the maps folder (`~/maps`): list, preview, save the map being built, activate, rename, delete (to a trash, reversible), restore.

It is standard library only and runs on the Jetson's Python 3.6.

## Safety

- It listens on **127.0.0.1 only**. You reach it through the SSH tunnel the console opens. Nothing on the LAN can talk to it.
- Every request needs the secret token in `~/.jetbot/token` (created on first start, mode 600). The console fetches it over SSH.
- It offers a **fixed set of actions**. Map names must match `^[A-Za-z0-9][A-Za-z0-9_-]{0,47}$`. No request value is ever put into a shell
  command, and it can only touch the maps folder.
- It **never publishes to ROS and cannot move the robot.** Motion still needs rosbridge and the console's MOTION switch.
- Maps are **never overwritten**. Saving to an existing name is refused. Deleting moves the map to `~/maps/.trash/` and can be undone.
- **rosbridge has no authentication and listens on every interface.** The manager therefore refuses to start it unless
  `jetbot-firewall.service` is active (it blocks outside access to port 9090). The install script sets that up.
- Starting navigation uses the real `/cmd_vel`. That is intended (the console gates motion), but it means a started navigation stack
  *can* drive the robot as soon as something publishes a goal or velocity. Keep the console's MOTION switch off unless you are supervising.

## Install (once, needs sudo)

On the Jetson, from the repository checkout (or the copy in `~/jetbot_manager/`):

    DRY_RUN=1 bash ~/jetbot_manager/install.sh      # preview: shows the units it would install, changes nothing
    sudo bash ~/jetbot_manager/install.sh           # installs and starts both services

This installs two systemd units:

| Unit | What it does |
|---|---|
| `jetbot-firewall.service` | Adds the iptables rule that drops non-local connections to port 9090 (IPv4 and IPv6). Starts at power-on. |
| `jetbot-manager.service` | Runs `jetbot_manager.py` as your user. Starts after the firewall unit and requires it. |

It does not touch ROS, the launch files or any map. Check it:

    curl -s http://127.0.0.1:9091/ping
    systemctl status jetbot-firewall.service jetbot-manager.service

Remove it with `sudo bash ~/jetbot_manager/uninstall.sh`.

Without the install the manager can still be run by hand (`python3 jetbot_manager.py`), but then you must apply the firewall rule
yourself (see `gui/robot_side/ROBOT_SETUP.md`, section 3), and the rule is lost on every reboot.

## Configuration

Defaults are at the top of `jetbot_manager.py` (`DEFAULTS`). The ones that matter:

| Key | Default | Meaning |
|---|---|---|
| `port` | 9091 | HTTP port (localhost only) |
| `maps_dir` | `~/maps` | where maps live |
| `linear_correction` / `angular_correction` | 1.15 / 0.9 | odometry calibration sent to the chassis at start (`docs/bringup_findings_2026-11-02.md`) |
| `map_extent_m` | 8.0 | half-width of the mapped area when mapping, in metres |
| `require_firewall_unit` | true | refuse to start rosbridge unless the firewall unit is active |
| `autostart_base` | true | start rosbridge when the manager starts |

They can be read and changed through `GET`/`POST /config`.

## API

All endpoints are on `http://127.0.0.1:9091` and, except `/ping`, need the header `X-Jetbot-Token: <token>`. Bodies are JSON.

| Method and path | Body | What it does |
|---|---|---|
| `GET /ping` | | liveness check, no token |
| `GET /status` | | mode (idle / navigation / mapping), running stack, ROS master and rosbridge ports, firewall, active map, host uptime / load / disk, last exit |
| `GET /maps` | | maps (with size and modified time), the active map, and the trash |
| `GET /maps/<name>/image` | | the map's PGM image, for the preview |
| `GET /logs?name=stack&lines=200` | | tail of a log |
| `POST /start` | `{"mode":"navigation","map":"<name>","initial_pose":{"x","y","a"}}` or `{"mode":"mapping","extent":8}`; add `"replace":true` to swap a running stack | start a stack |
| `POST /stop` | | stop the running stack |
| `POST /base/start`, `/base/stop` | | start or stop only rosbridge |
| `POST /maps/save` | `{"name":"<new name>"}` | save the map being built; refused if the name exists |
| `POST /maps/activate` | `{"name":"<name>"}` | make a map the one navigation uses |
| `POST /maps/rename` | `{"name","new_name"}` | rename (refused if the new name exists) |
| `POST /maps/delete` | `{"name"}` | move to the trash |
| `POST /maps/restore` | `{"entry":"<trash entry>"}` | bring a trashed map back |

## Tests

They run on the Mac or the Jetson without ROS (the launches are replaced by harmless stand-in processes):

    python3 -I -m unittest discover -s manager -p "test_*.py" -v      # from the repository root

## Not covered yet

- A real emergency stop. Stopping the stack stops the nodes, but a velocity mux on the robot (`twist_mux`, needs sudo) is the proper fix.
- The first live run of Start navigation, Map a new area, Save, and switching maps with the robot present. The read-only endpoints,
  rename, delete and restore were exercised on the Jetson on a spare port with a throwaway map.
