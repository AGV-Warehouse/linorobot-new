#!/usr/bin/env python3
# Object boxes: makes the operator app's object boxes real obstacles.
#
# The app (agvapp) groups furniture-sized blobs of the map into boxes. The
# raw map only has the legs, so the planner would happily route between the
# legs of a table. This node stamps the app's boxes into a copy of /map and
# publishes it latched on /map_nav, which the GLOBAL costmap's static layer
# uses (navigation.yaml: static_layer.map_topic). AMCL keeps the raw /map.
#
#   /map           nav_msgs/OccupancyGrid          in  (latched, map_server)
#   /object_boxes  std_msgs/Float32MultiArray      in  [xmin, ymin, xmax, ymax]
#                                                      per box, map frame, m
#   /map_nav       nav_msgs/OccupancyGrid          out (latched)
#
# With no app connected /map_nav is /map unchanged. Boxes are kept in memory
# until replaced, so they survive the app closing but NOT a nav restart.
# If this node is not running the global costmap gets no map and every goal
# fails to plan -- loud on purpose.
#
# Self-check without ROS:  python3 object_boxes.py --selftest

import math
import sys

MAX_BOXES = 500  # a sane map has tens; more is a malformed message


def stamp_boxes(data, width, height, resolution, origin_x, origin_y, rects):
    """Returns a copy of `data` (row-major, row 0 = min y) with every cell
    touched by a rect [xmin, ymin, xmax, ymax, ...] set to 100. Malformed
    input (bad length, non-finite numbers) stamps nothing."""
    out = list(data)
    if len(rects) % 4 or len(rects) > 4 * MAX_BOXES:
        return out
    if not all(math.isfinite(v) for v in rects):
        return out
    eps = 1e-4  # float32 edges sit a hair past the cell boundary
    for i in range(0, len(rects), 4):
        x0, y0, x1, y1 = rects[i:i + 4]
        c0 = max(0, math.floor((x0 - origin_x) / resolution + eps))
        r0 = max(0, math.floor((y0 - origin_y) / resolution + eps))
        c1 = min(width, math.ceil((x1 - origin_x) / resolution - eps))
        r1 = min(height, math.ceil((y1 - origin_y) / resolution - eps))
        for row in range(r0, r1):
            for col in range(c0, c1):
                out[row * width + col] = 100
    return out


def selftest():
    grid = [0] * 100  # 10 x 10 cells of 0.5 m, origin (-1, -1)
    grid[0] = -1
    out = stamp_boxes(grid, 10, 10, 0.5, -1.0, -1.0, [0.0, 0.0, 1.0, 0.5])
    assert [i for i, v in enumerate(out) if v == 100] == [22, 23], out
    assert out[0] == -1 and grid[22] == 0  # rest untouched, input not mutated
    # off-map rects are clipped, garbage is ignored
    assert stamp_boxes(grid, 10, 10, 0.5, -1.0, -1.0, [-9, -9, 9, 9]).count(100) == 100
    assert stamp_boxes(grid, 10, 10, 0.5, -1.0, -1.0, [0.0, 0.0, 1.0]) == grid
    assert stamp_boxes(grid, 10, 10, 0.5, -1.0, -1.0, [0.0, 0.0, float('nan'), 1.0]) == grid
    print('object_boxes selftest: PASS')


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from nav_msgs.msg import OccupancyGrid
    from std_msgs.msg import Float32MultiArray

    latched = QoSProfile(
        depth=1,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        reliability=ReliabilityPolicy.RELIABLE,
    )

    class ObjectBoxes(Node):
        def __init__(self):
            super().__init__('object_boxes')
            self.map = None
            self.rects = []
            self.pub = self.create_publisher(OccupancyGrid, 'map_nav', latched)
            self.create_subscription(OccupancyGrid, 'map', self.on_map, latched)
            # Latched on this side too: rosbridge publishers are transient
            # local, so a box list sent before DDS discovery finished still
            # arrives.
            self.create_subscription(
                Float32MultiArray, 'object_boxes', self.on_boxes, latched)

        def on_map(self, msg):
            self.map = msg
            self.publish()

        def on_boxes(self, msg):
            self.rects = list(msg.data)
            self.get_logger().info(f'{len(self.rects) // 4} object boxes from the app')
            self.publish()

        def publish(self):
            if self.map is None:
                return
            info = self.map.info
            out = OccupancyGrid()
            out.header = self.map.header
            out.info = info
            out.data = stamp_boxes(
                self.map.data, info.width, info.height, info.resolution,
                info.origin.position.x, info.origin.position.y, self.rects)
            self.pub.publish(out)

    rclpy.init()
    try:
        rclpy.spin(ObjectBoxes())
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    selftest() if '--selftest' in sys.argv else main()
