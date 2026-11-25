#!/usr/bin/env bash
# Install the boot services. Run with: sudo ./install.sh
set -euo pipefail
cd "$(dirname "$0")"

install -m 644 99-rplidar.rules /etc/udev/rules.d/
udevadm control --reload-rules && udevadm trigger --subsystem-match=tty

install -m 644 agv-bringup.service agv-navigation.service claude-remote.service /etc/systemd/system/
systemctl daemon-reload
systemctl enable agv-bringup.service agv-navigation.service claude-remote.service

echo "Installed and enabled. They start on next boot, or now with:"
echo "  sudo systemctl start agv-bringup agv-navigation claude-remote"
