#!/usr/bin/env python3
# How well does the live scan line up with the map at AMCL's current pose?
# Run on the Jetson with the stack up (robot.launch.py + navigation.launch.py):
#   python3 scan_map_fit.py
# Prints AMCL's pose and the share of /scan points within 10 cm of a map wall.
# 2026-11-11, mislocalized (robot drawn on a wall, every plan failed): 34%.
# A correct pose should score far higher -- record the well-localized value
# in TROUBLESHOOTING.md the first time you see it. Low score: Set Pose.
# Read-only: subscribes /map, /scan and TF; publishes nothing.
import math, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener
rclpy.init(); n = Node('scanfit'); buf = Buffer(); TransformListener(buf, n)
st = {}
n.create_subscription(OccupancyGrid, 'map', lambda m: st.setdefault('map', m),
    QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE))
n.create_subscription(LaserScan, 'scan', lambda s: st.__setitem__('scan', s), 10)
end = n.get_clock().now().nanoseconds + 12e9
while rclpy.ok() and n.get_clock().now().nanoseconds < end and not ('map' in st and 'scan' in st and buf.can_transform('map', st['scan'].header.frame_id, rclpy.time.Time())):
    rclpy.spin_once(n, timeout_sec=0.2)
m, s = st['map'], st['scan']; info = m.info
t = buf.lookup_transform('map', s.header.frame_id, rclpy.time.Time()).transform
b = buf.lookup_transform('map', 'base_footprint', rclpy.time.Time()).transform
yaw = lambda q: math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
ty = yaw(t.rotation)
def cell(x, y):
    i = int((x-info.origin.position.x)/info.resolution); j = int((y-info.origin.position.y)/info.resolution)
    return m.data[j*info.width+i] if 0 <= i < info.width and 0 <= j < info.height else -2
def near_occ(x, y, r=2):
    i0 = int((x-info.origin.position.x)/info.resolution); j0 = int((y-info.origin.position.y)/info.resolution)
    for dj in range(-r, r+1):
        for di in range(-r, r+1):
            i, j = i0+di, j0+dj
            if 0 <= i < info.width and 0 <= j < info.height and m.data[j*info.width+i] > 50: return True
    return False
hits = tot = 0
for k, r in enumerate(s.ranges):
    if not (s.range_min < r < min(s.range_max, 8.0)): continue
    a = s.angle_min + k*s.angle_increment + ty
    x, y = t.translation.x + r*math.cos(a), t.translation.y + r*math.sin(a)
    tot += 1; hits += near_occ(x, y)
bx, by = b.translation.x, b.translation.y
print(f"robot pose (map): x={bx:.2f} y={by:.2f} yaw={math.degrees(yaw(b.rotation)):.0f} deg")
print(f"map cell under robot center: {cell(bx, by)} (0 free, 100 occupied, -1 unknown)")
print(f"scan points within 10 cm of a map wall: {hits}/{tot} = {100*hits/max(tot,1):.0f}%")
