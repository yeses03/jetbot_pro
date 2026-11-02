#!/usr/bin/env bash
# Read-only snapshot of the JetBot's ROS state, collected over SSH.
#
# Only subscribes, reads parameters, and reads files. It never publishes to
# /cmd_vel or any motor topic, sends goals, restarts nodes, or writes on the
# Jetson. Usage: tools/jetson_readonly_diag.sh [ssh-alias] [output-dir]
set -u

HOST="${1:-jetson}"
OUT="${2:-${TMPDIR:-/tmp}/jetbot_diag_$(date +%Y%m%d_%H%M%S)}"
ROS='source /opt/ros/melodic/setup.bash && source ~/catkin_ws/devel/setup.bash'
mkdir -p "$OUT"

# run NAME 'remote command' -> saves output to $OUT/NAME.txt
run() {
  local name="$1"; shift
  ssh -o BatchMode=yes -o ConnectTimeout=8 "$HOST" "$@" > "$OUT/$name.txt" 2>&1
  echo "collected $name"
}

run git_status   'cd ~/catkin_ws/src/jetbot_pro && git rev-parse --abbrev-ref HEAD && git log --oneline | head -3 && git status --short'
run git_staged   'cd ~/catkin_ws/src/jetbot_pro && git diff --cached --ignore-space-at-eol'
run git_unstaged 'cd ~/catkin_ws/src/jetbot_pro && git diff --ignore-space-at-eol'
run load         'uptime; ps -eo pid,pcpu,etime,cmd --sort=-pcpu | head -8 | cut -c1-120'
run nodes        "$ROS && timeout 8 rosnode list && timeout 8 rostopic list"
for t in odom_raw odom_combined odom imu; do
  run "echo_$t" "$ROS && timeout 10 rostopic echo -n 1 /$t"
  run "hz_$t"   "$ROS && timeout 7 rostopic hz /$t"
done
run ekf_status   "$ROS && timeout 10 rosservice call /robot_pose_ekf/get_status"
run tf_map_odom  "$ROS && timeout 4 rosrun tf tf_echo map odom"
run tf_odom_base "$ROS && timeout 4 rosrun tf tf_echo odom base_footprint"
run tf_odom_laser "$ROS && timeout 4 rosrun tf tf_echo odom laser_frame"
run map_info     "$ROS && timeout 6 rostopic echo -n 1 /map_metadata"
run params       "$ROS && rosparam get /slam_gmapping; rosparam get /robot_pose_ekf; rosparam get /jetbot"

echo "saved to $OUT"
