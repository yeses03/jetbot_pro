#!/usr/bin/env bash
# Installs the JetBot manager and the rosbridge firewall as systemd services, so both start at power-on.
# Run ONCE on the Jetson:   sudo bash ~/jetbot_manager/install.sh        (preview first:  DRY_RUN=1 bash ~/jetbot_manager/install.sh)
# It does not touch ROS, the robot's launch files or any map.
set -euo pipefail

DRY="${DRY_RUN:-0}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ "$DRY" = "1" ]; then
  USER_NAME="${SUDO_USER:-$(id -un)}"
else
  [ "$(id -u)" = "0" ] || { echo "Run with sudo:  sudo bash $0" >&2; exit 1; }
  USER_NAME="${SUDO_USER:-}"
  [ -n "$USER_NAME" ] && [ "$USER_NAME" != "root" ] || { echo "Run it with sudo from your normal user (not as root directly)." >&2; exit 1; }
fi
[[ "$USER_NAME" =~ ^[a-z_][a-z0-9_-]*$ ]] || { echo "unexpected user name: $USER_NAME" >&2; exit 1; }
HOME_DIR="$(getent passwd "$USER_NAME" 2>/dev/null | cut -d: -f6 || true)"
[ -n "$HOME_DIR" ] || HOME_DIR="$(eval echo "~$USER_NAME")"
UNIT_DIR="/etc/systemd/system"
run() { if [ "$DRY" = "1" ]; then echo "DRY-RUN: $*"; else "$@"; fi; }

echo "User: $USER_NAME   Install dir: $HERE   Units: $UNIT_DIR"
[ -f "$HERE/jetbot_manager.py" ] || { echo "jetbot_manager.py not found next to this script" >&2; exit 1; }
python3 -c "import sys; assert sys.version_info >= (3, 6)" || { echo "needs Python 3.6+" >&2; exit 1; }
if [ "$DRY" != "1" ]; then python3 -m py_compile "$HERE/jetbot_manager.py"; fi
mkdir -p "$HOME_DIR/.jetbot" "$HOME_DIR/maps" "$HOME_DIR/logs"
[ "$DRY" = "1" ] || chown -R "$USER_NAME":"$USER_NAME" "$HOME_DIR/.jetbot"

for unit in jetbot-firewall.service jetbot-manager.service; do
  sed -e "s#@INSTALL_DIR@#$HERE#g" -e "s#@USER@#$USER_NAME#g" -e "s#@HOME@#$HOME_DIR#g" "$HERE/$unit" > "/tmp/$unit.rendered"
  if [ "$DRY" = "1" ]; then
    echo "DRY-RUN: would install /tmp/$unit.rendered as $UNIT_DIR/$unit"
    grep -E "^(ExecStart|User|Requires|Environment)=" "/tmp/$unit.rendered" | sed 's/^/    /'
  else
    install -m 644 "/tmp/$unit.rendered" "$UNIT_DIR/$unit"
  fi
  rm -f "/tmp/$unit.rendered"
done

run systemctl daemon-reload
run systemctl enable --now jetbot-firewall.service
if [ "$DRY" != "1" ]; then
  iptables -C INPUT -p tcp -m tcp --dport 9090 ! -i lo -j DROP && echo "OK: firewall rule for port 9090 is active" \
    || { echo "FAILED: the firewall rule is not active; not starting the manager" >&2; exit 1; }
fi
run systemctl enable --now jetbot-manager.service
if [ "$DRY" != "1" ]; then
  sleep 3
  systemctl --no-pager --lines=0 status jetbot-firewall.service jetbot-manager.service || true
  echo
  echo "Done. The manager token (the console fetches it over SSH, you do not need to copy it) is in $HOME_DIR/.jetbot/token"
  echo "Check:  curl -s http://127.0.0.1:9091/ping"
fi
