#!/usr/bin/env bash
# Removes the two systemd services. Leaves maps, logs and ~/.jetbot alone. Run:  sudo bash ~/jetbot_manager/uninstall.sh
set -euo pipefail
[ "$(id -u)" = "0" ] || { echo "Run with sudo" >&2; exit 1; }
systemctl disable --now jetbot-manager.service 2>/dev/null || true
systemctl disable --now jetbot-firewall.service 2>/dev/null || true
rm -f /etc/systemd/system/jetbot-manager.service /etc/systemd/system/jetbot-firewall.service
systemctl daemon-reload
echo "Removed. The firewall rule is gone too: do not start rosbridge on this robot without re-adding it."
