#!/usr/bin/env python3
"""Preflight check before sending a /dock_robot goal.

Grabs one detected_dock_pose and the robot's odom pose, then judges the
entry geometry: the docking server drives IN ALONG THE POSE'S +x AXIS, so
the robot must already stand near that line, roughly facing the cart.

Run (stack + detector up, robot parked):
    python3 linorobot2_navigation/scripts/dock_preflight.py

PASS prints the exact `ros2 action send_goal` command to paste.
"""
import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped
import tf2_ros

# Gate values chosen against the ~0.60 m short-side leg gap of the
# practice table: 2-3 cm of lateral margin per side once inside.
MAX_LATERAL_OFF_LINE = 0.10   # m, robot distance from the entry line
MAX_AXIS_MISMATCH = 10.0      # deg, |entry yaw - bearing robot->center|
MAX_HEADING_OFF = 25.0        # deg, robot heading vs entry axis (soft)
DIST_RANGE = (0.9, 2.5)       # m, robot to cart center


class Preflight(Node):
    def __init__(self):
        super().__init__('dock_preflight')
        self.pose = None
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.create_subscription(
            PoseStamped, 'detected_dock_pose', self.cb, 10)

    def cb(self, msg):
        self.pose = msg


def main():
    rclpy.init()
    node = Preflight()
    deadline = node.get_clock().now() + Duration(seconds=6)
    while rclpy.ok() and node.pose is None \
            and node.get_clock().now() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    if node.pose is None:
        print('FAIL: no detected_dock_pose within 6 s. Is the detector '
              '(terminal 3) running and locked? It must STAY running '
              'through the whole dock.')
        return 1

    p = node.pose.pose
    cx, cy = p.position.x, p.position.y
    yaw = 2.0 * math.atan2(p.orientation.z, p.orientation.w)

    try:
        t = node.tf_buffer.lookup_transform(
            node.pose.header.frame_id, 'base_footprint', Time(),
            timeout=Duration(seconds=3))
    except tf2_ros.TransformException as e:
        print('FAIL: no odom->base_footprint TF (%s). Is bringup up?' % e)
        return 1
    rx, ry = t.transform.translation.x, t.transform.translation.y
    q = t.transform.rotation
    ryaw = 2.0 * math.atan2(q.z, q.w)

    def wrap(a):
        return (a + math.pi) % (2 * math.pi) - math.pi

    dist = math.hypot(cx - rx, cy - ry)
    bearing = math.atan2(cy - ry, cx - rx)
    axis_mismatch = math.degrees(abs(wrap(yaw - bearing)))
    heading_off = math.degrees(abs(wrap(yaw - ryaw)))
    # Perpendicular distance of the robot from the entry line
    # (line through the cart center along the entry yaw).
    lateral = abs(math.cos(yaw) * (ry - cy) - math.sin(yaw) * (rx - cx))

    print('cart center : (%.3f, %.3f)  entry yaw %.1f deg' %
          (cx, cy, math.degrees(yaw)))
    print('robot       : (%.3f, %.3f)  heading   %.1f deg' %
          (rx, ry, math.degrees(ryaw)))
    print('distance    : %.2f m   (want %.1f-%.1f)' %
          (dist, *DIST_RANGE))
    print('off the line: %.2f m   (max %.2f)' %
          (lateral, MAX_LATERAL_OFF_LINE))
    print('axis vs bearing: %.1f deg (max %.0f) | heading vs axis: '
          '%.1f deg (max %.0f)' %
          (axis_mismatch, MAX_AXIS_MISMATCH, heading_off, MAX_HEADING_OFF))

    problems = []
    if not DIST_RANGE[0] <= dist <= DIST_RANGE[1]:
        problems.append('distance out of range — repark %.1f-%.1f m from '
                        'the cart center' % DIST_RANGE)
    if lateral > MAX_LATERAL_OFF_LINE:
        problems.append('robot is %.2f m off the entry line — shift '
                        'sideways until centered on the leg gap' % lateral)
    if axis_mismatch > MAX_AXIS_MISMATCH:
        problems.append('entry axis does not point at the robot — repark '
                        'square-on, then RESTART the detector to re-lock')
    if heading_off > MAX_HEADING_OFF:
        problems.append('robot heading far off the entry axis — rotate '
                        'to face the cart')

    if problems:
        print('\nFAIL:')
        for pr in problems:
            print('  - %s' % pr)
        print('Fix, wait ~2 s for the filter, run this again.')
        return 1

    print('\nPASS — send this (Ctrl-C on it aborts the dock):\n')
    print('ros2 action send_goal /dock_robot nav2_msgs/action/DockRobot '
          '"{use_dock_id: false, dock_type: simple_charging_dock, '
          'navigate_to_staging_pose: false, dock_pose: {header: {frame_id: '
          '%s}, pose: {position: {x: %.4f, y: %.4f}, orientation: '
          '{z: %.4f, w: %.4f}}}}" --feedback' %
          (node.pose.header.frame_id, cx, cy,
           math.sin(yaw / 2.0), math.cos(yaw / 2.0)))
    return 0


if __name__ == '__main__':
    code = main()
    rclpy.shutdown()
    sys.exit(code)
