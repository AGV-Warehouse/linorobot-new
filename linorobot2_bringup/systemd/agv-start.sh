#!/usr/bin/env bash
# Boot launcher used by the agv-*.service units (see README.md here).
#   agv-start.sh bringup     -> robot.launch.py slam:=false (waits for USB devices)
#   agv-start.sh navigation  -> navigation.launch.py on map_new (waits for odom)
set -eo pipefail

source /opt/ros/jazzy/setup.bash
source "$HOME/linorobot2_ws/install/setup.bash"
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET

REPO="$HOME/linorobot2_ws/src/linorobot-new"
MAP="${AGV_MAP:-$REPO/linorobot2_navigation/maps/map_new.yaml}"
LIDAR=/dev/rplidar
[ -e "$LIDAR" ] || LIDAR=/dev/ttyUSB0   # until the udev rule is installed

wait_for() { # path, label
  for _ in $(seq 120); do [ -e "$1" ] && return 0; sleep 1; done
  echo "agv-start: $2 ($1) never appeared" >&2; exit 1
}

case "$1" in
  bringup)
    wait_for /dev/ttyACM0 "Nucleo"
    wait_for "$LIDAR" "lidar"
    if fuser /dev/ttyACM0 >/dev/null 2>&1; then
      echo "agv-start: /dev/ttyACM0 already in use (manual micro-ROS agent?)" >&2
      fuser -v /dev/ttyACM0 >&2 || true
      exit 1
    fi
    exec ros2 launch linorobot2_bringup robot.launch.py slam:=false lidar_port:="$LIDAR"
    ;;
  navigation)
    # Nav2 aborts its lifecycle bringup if odom isn't there yet, and never retries.
    echo "agv-start: waiting for /odom/unfiltered"
    until timeout 10 ros2 topic echo /odom/unfiltered --once >/dev/null 2>&1; do sleep 2; done
    echo "agv-start: odom flowing, starting navigation with $MAP"
    exec ros2 launch linorobot2_navigation navigation.launch.py map:="$MAP"
    ;;
  *)
    echo "usage: $0 bringup|navigation" >&2; exit 2 ;;
esac
