#!/usr/bin/env python3
"""
Live SLAM map viewer over the network -- no rviz needed.

Subscribes to /map (nav_msgs/OccupancyGrid) and the robot pose (/pose from
slam_toolbox while mapping, /amcl_pose from AMCL while navigating), renders a
PNG, and serves a small auto-refreshing web page. Open it in any browser:

  - On the same LAN:        http://192.168.1.59:8000
  - Through an SSH tunnel:   ssh -L 8000:localhost:8000 jetson1@192.168.1.59
                            then open http://localhost:8000 on your laptop

Hover over the map to read map-frame coordinates in meters; click anywhere to
get a ready-to-paste `ros2 action send_goal` command for that spot. The robot
is drawn at its actual footprint size (550mm x 550mm) with a heading line.

Cart parking spots
------------------
The page can also draw fixed-size cart boxes on the map and save them to
`<map>.spots.json` next to the map (see --spots). A spot is the map-frame
pose the robot ends up at: this robot drives UNDER a cart and carries it on
top, so the cart box and the robot footprint are concentric and one pose
describes both. Spots are only recorded and drawn -- the page never commands
the robot, it just hands you the `ros2 action send_goal` line for a spot.

The map PNG is encoded once per /map message and cached; the robot footprint
is drawn client-side on a canvas overlay from /meta.json, so pose updates are
cheap and frequent while the heavy image work only happens when the map
actually changes.

Everything here is unauthenticated and bound to 0.0.0.0 -- anyone on the LAN
can set the initial pose and edit parking spots. That is the same trust model
the page has always had; keep it off untrusted networks.

robot.launch.py starts this automatically (map_viewer:=false to disable).
To run it standalone on the Jetson (after sourcing ROS):
  source /opt/ros/jazzy/setup.bash
  source ~/linorobot2_ws/install/setup.bash
  python3 ~/Desktop/linorobot2/linorobot2_bringup/scripts/map_viewer.py
"""

import io
import json
import math
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np
from PIL import Image, ImageDraw

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseWithCovarianceStamped

PORT = 8000
UPSCALE = 4          # pixels per map cell, makes the image easier to see
MAP_POLL_MS = 500    # browser meta poll; map image reloads only when it changed
ROBOT_HALF = 0.275   # meters: half of the 550mm x 550mm square footprint

# Default cart footprint: length is along the spot's heading, width across it.
# Placeholder until the real carts are measured -- editable live on the page
# and stored in the spots file, so fixing it never means editing this code.
CART_LEN = 0.80
CART_WIDTH = 0.60

SPOTS_VERSION = 1    # bump if the on-disk schema ever changes incompatibly
SPOTS_SUFFIX = '.spots.json'
MAX_BODY = 64 * 1024  # cap on a POST body; spot edits are tiny


class SpotStore:
    """Cart parking spots, persisted as JSON next to the map.

    Lives in linorobot2_navigation/maps/<map>.spots.json so spots are
    versioned in git alongside the map they were drawn on -- they are map
    coordinates, not features, so re-SLAMming the map invalidates them.

    Has its own lock: the HTTP server runs on a daemon thread while
    rclpy.spin() owns the main one.
    """

    def __init__(self, path=None, logger=None):
        self._lock = threading.Lock()
        self._log = logger
        self._rev = 0
        self._seq = 0
        self._cart = {'length': CART_LEN, 'width': CART_WIDTH}
        self._spots = []
        self._note = ''
        self._path = self._resolve(path) if path else None
        self._map = ''
        if self._path:
            base = os.path.basename(self._path)
            self._map = base[:-len(SPOTS_SUFFIX)] if base.endswith(SPOTS_SUFFIX) else base
            self._load()
        else:
            self._note = 'no --spots path: parking spots are not saved'
            self._info(self._note)

    # ---- paths & persistence -------------------------------------------

    @staticmethod
    def _resolve(requested):
        """Follow the map yaml's symlink so spots land in the source tree.

        The viewer runs out of the colcon install space, where
        `install(DIRECTORY maps ...)` leaves one symlink per file pointing
        back at the source checkout. The spots file usually does not exist
        yet, so realpath() on it would just hand back the install path --
        resolve the sibling <map>.yaml (which does exist, and is the
        symlink) and write next to wherever that really lives.
        """
        path = os.path.abspath(os.path.expanduser(requested))
        head, base = os.path.split(path)
        if base.endswith(SPOTS_SUFFIX):
            sibling = os.path.join(head, base[:-len(SPOTS_SUFFIX)] + '.yaml')
            if os.path.exists(sibling):
                head = os.path.dirname(os.path.realpath(sibling))
        return os.path.join(head, base)

    def _info(self, msg):
        if self._log is not None:
            self._log.info(msg)

    def _warn(self, msg):
        if self._log is not None:
            self._log.warning(msg)

    def _load(self):
        if not os.path.exists(self._path):
            self._info(f'Parking spots: {self._path} (new file, 0 loaded)')
            return
        try:
            with open(self._path) as f:
                doc = json.load(f)
            cart = doc.get('cart') or {}
            self._cart = {
                'length': self._dim(cart.get('length', CART_LEN)),
                'width': self._dim(cart.get('width', CART_WIDTH)),
            }
            self._spots = [self._clean(s) for s in doc.get('spots', [])]
            saved_map = doc.get('map', '')
            if saved_map and self._map and saved_map != self._map:
                # Spots are map coordinates. A different map means a
                # different origin, so drawing them would be a lie.
                self._note = (f'spots were saved for map "{saved_map}" but '
                              f'this file is "{self._map}" -- check them')
                self._warn(self._note)
        except (OSError, ValueError, KeyError, TypeError) as e:
            # A corrupt file must not take the page down; refuse to write
            # over it either, or one bad parse silently eats every spot.
            self._spots = []
            self._note = f'could not read {self._path}: {e} -- not saving'
            self._warn(self._note)
            self._path = None
            return
        self._info(f'Parking spots: {self._path} ({len(self._spots)} loaded)')

    def _save_locked(self):
        if not self._path:
            return
        doc = {'version': SPOTS_VERSION, 'map': self._map,
               'cart': self._cart, 'spots': self._spots}
        body = json.dumps(doc, indent=2, sort_keys=True) + '\n'
        for attempt in (self._path, self._fallback()):
            if not attempt:
                break
            try:
                os.makedirs(os.path.dirname(attempt), exist_ok=True)
                tmp = attempt + '.tmp'
                with open(tmp, 'w') as f:
                    f.write(body)
                os.replace(tmp, attempt)   # atomic: no torn file on power cut
                if attempt != self._path:
                    self._path = attempt
                    self._note = f'map dir not writable -- saving to {attempt}'
                    self._warn(self._note)
                return
            except OSError as e:
                self._warn(f'could not write {attempt}: {e}')
        self._note = 'parking spots could NOT be saved to disk'
        self._path = None

    def _fallback(self):
        """~/.ros/<map>.spots.json, the same place pose_keeper.py writes."""
        if not self._path:
            return None
        alt = os.path.expanduser(os.path.join('~/.ros', os.path.basename(self._path)))
        return None if alt == self._path else alt

    # ---- validation -----------------------------------------------------

    @staticmethod
    def _num(value, what):
        try:
            out = float(value)
        except (TypeError, ValueError):
            raise ValueError(f'{what} must be a number')
        if not math.isfinite(out):
            raise ValueError(f'{what} must be finite')
        return round(out, 3)

    @classmethod
    def _dim(cls, value):
        out = cls._num(value, 'size')
        if not 0.05 <= out <= 5.0:
            raise ValueError('cart sizes must be between 0.05 m and 5 m')
        return out

    @staticmethod
    def _label(value, fallback):
        name = str(value if value is not None else '').strip()
        return (name or fallback)[:40]

    @classmethod
    def _clean(cls, spot):
        """Normalize a spot read off disk; raises on anything unusable."""
        return {
            'id': str(spot['id'])[:40],
            'name': cls._label(spot.get('name'), str(spot['id'])),
            'x': cls._num(spot['x'], 'x'),
            'y': cls._num(spot['y'], 'y'),
            'yaw': cls._num(spot['yaw'], 'yaw'),
            'length': cls._dim(spot.get('length', CART_LEN)),
            'width': cls._dim(spot.get('width', CART_WIDTH)),
            'updated': str(spot.get('updated', ''))[:32],
        }

    def _find_locked(self, spot_id):
        for spot in self._spots:
            if spot['id'] == str(spot_id):
                return spot
        raise KeyError(f'no such spot: {spot_id}')

    # ---- reads ----------------------------------------------------------

    def rev(self):
        with self._lock:
            return self._rev

    def as_json(self):
        with self._lock:
            return json.dumps({'rev': self._rev, 'map': self._map,
                               'cart': self._cart, 'spots': self._spots,
                               'saved': bool(self._path), 'note': self._note,
                               'path': self._path or ''}).encode()

    # ---- writes ---------------------------------------------------------

    def add(self, req):
        with self._lock:
            self._seq += 1
            spot = self._clean({
                'id': f's{int(time.time() * 1000)}_{self._seq}',
                'name': self._label(req.get('name'), f'Spot {len(self._spots) + 1}'),
                'x': req['x'], 'y': req['y'], 'yaw': req['yaw'],
                'length': req.get('length', self._cart['length']),
                'width': req.get('width', self._cart['width']),
                'updated': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
            })
            self._spots.append(spot)
            self._rev += 1
            self._save_locked()
            return spot

    def update(self, req):
        with self._lock:
            spot = self._find_locked(req['id'])
            for key in ('x', 'y', 'yaw'):
                if key in req:
                    spot[key] = self._num(req[key], key)
            for key in ('length', 'width'):
                if key in req:
                    spot[key] = self._dim(req[key])
            if 'name' in req:
                spot['name'] = self._label(req['name'], spot['name'])
            spot['updated'] = time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime())
            self._rev += 1
            self._save_locked()
            return spot

    def delete(self, req):
        with self._lock:
            spot = self._find_locked(req['id'])
            self._spots.remove(spot)
            self._rev += 1
            self._save_locked()
            return {'ok': True, 'id': spot['id']}

    def set_cart(self, req):
        with self._lock:
            self._cart = {'length': self._dim(req['length']),
                          'width': self._dim(req['width'])}
            self._rev += 1
            self._save_locked()
            return {'ok': True, 'cart': self._cart}


class MapRenderer(Node):
    def __init__(self, map_file=None, spots_path=None):
        super().__init__('map_viewer')
        self._lock = threading.Lock()
        self._png = self._placeholder_png()
        self._meta = None   # (res, ox, oy, w, h)
        self._pose = None   # (x, y, yaw) in map frame
        self._seq = 0       # bumps on every new map so the browser knows to reload
        self._slam_pose_time = 0.0
        self._plan = None       # [[x,y], ...] downsampled current global plan
        self._goal = None       # (x, y, yaw) = last pose of the current plan
        self._plan_time = 0.0   # monotonic time of the last /plan message

        # Cart parking spots, saved next to the map (see SpotStore).
        self.spots = SpotStore(spots_path, logger=self.get_logger())

        # Preload a saved map (map_saver_cli yaml+pgm) so the page has content
        # with no SLAM or map_server running. A live /map still overrides it.
        if map_file:
            self._load_map_file(map_file)

        # /map is published with TRANSIENT_LOCAL (latched) durability.
        map_qos = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, '/map', self._on_map, map_qos)
        # slam_toolbox publishes /pose while mapping; AMCL publishes /amcl_pose
        # while navigating. SLAM wins when both are alive (a stray AMCL running
        # during mapping publishes garbage poses); AMCL is only used when no
        # SLAM pose has arrived recently.
        self.create_subscription(
            PoseWithCovarianceStamped, '/pose',
            lambda m: self._on_pose(m, slam=True), 10)
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose',
            lambda m: self._on_pose(m, slam=False), 10)
        # Nav2's planner_server publishes the current global plan on /plan
        # (republished ~1 Hz while a goal is active). Rendered on the page as
        # a path line with a blue footprint at the destination.
        self.create_subscription(Path, '/plan', self._on_plan, 10)
        # RViz-less "2D Pose Estimate": the web page can publish /initialpose
        # so AMCL can be (re)localized from the browser.
        self._initpose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)
        self.get_logger().info(f'Map viewer up. Browse to http://<jetson-ip>:{PORT}')

    def publish_initial_pose(self, x, y, yaw):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        # Same uncertainty RViz uses for its 2D Pose Estimate tool.
        msg.pose.covariance[0] = 0.25    # x
        msg.pose.covariance[7] = 0.25    # y
        msg.pose.covariance[35] = 0.0685  # yaw
        self._initpose_pub.publish(msg)
        self.get_logger().info(f'initial pose set: x={x:.2f} y={y:.2f} yaw={yaw:.2f}')

    def _on_pose(self, msg, slam):
        now = time.monotonic()
        with self._lock:
            if slam:
                self._slam_pose_time = now
            elif now - self._slam_pose_time < 10.0:
                return  # SLAM is alive; ignore AMCL
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        with self._lock:
            self._pose = (p.x, p.y, yaw)

    def _on_plan(self, msg):
        if not msg.poses:
            return
        # Downsample to <=150 points: plenty for drawing, tiny meta.json.
        step = max(1, len(msg.poses) // 150)
        pts = [[round(p.pose.position.x, 3), round(p.pose.position.y, 3)]
               for p in msg.poses[::step]]
        last = msg.poses[-1].pose
        pts.append([round(last.position.x, 3), round(last.position.y, 3)])
        q = last.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        with self._lock:
            self._plan = pts
            self._goal = (round(last.position.x, 3),
                          round(last.position.y, 3), round(yaw, 3))
            self._plan_time = time.monotonic()

    def _on_map(self, msg):
        w, h = msg.info.width, msg.info.height
        if w == 0 or h == 0:
            return
        res = msg.info.resolution
        ox, oy = msg.info.origin.position.x, msg.info.origin.position.y

        data = np.array(msg.data, dtype=np.int16).reshape(h, w)
        # Color map: unknown(-1)=gray, free(0)=white, occupied(100)=black.
        img = np.full((h, w), 205, dtype=np.uint8)        # unknown
        known = data >= 0
        img[known] = (255 * (1.0 - data[known] / 100.0)).astype(np.uint8)

        # OccupancyGrid origin is bottom-left; image origin is top-left -> flip rows.
        img = np.flipud(img)
        pil = Image.fromarray(img, mode='L').convert('RGB')
        pil = pil.resize((w * UPSCALE, h * UPSCALE), Image.NEAREST)

        # Encode ONCE per map message (compress_level=1: speed over size, it's
        # a LAN) and serve the cached bytes. The robot overlay is drawn in the
        # browser, so requests never trigger image work here.
        buf = io.BytesIO()
        pil.save(buf, format='PNG', compress_level=1)
        with self._lock:
            self._png = buf.getvalue()
            self._meta = (res, ox, oy, w, h)
            self._seq += 1

    def _load_map_file(self, yaml_path):
        import yaml as pyyaml
        with open(yaml_path) as f:
            info = pyyaml.safe_load(f)
        img_path = info['image']
        if not os.path.isabs(img_path):
            img_path = os.path.join(os.path.dirname(os.path.abspath(yaml_path)), img_path)
        # The saved pgm is already in image orientation (row 0 = top of map)
        # with the same gray levels the live renderer produces.
        pil = Image.open(img_path).convert('L').convert('RGB')
        w, h = pil.size
        pil = pil.resize((w * UPSCALE, h * UPSCALE), Image.NEAREST)
        buf = io.BytesIO()
        pil.save(buf, format='PNG', compress_level=1)
        ox, oy = float(info['origin'][0]), float(info['origin'][1])
        with self._lock:
            self._png = buf.getvalue()
            self._meta = (float(info['resolution']), ox, oy, w, h)
            self._seq += 1
        self.get_logger().info(f'Loaded saved map {yaml_path} ({w}x{h} cells)')

    def png(self):
        with self._lock:
            return self._png

    def meta_json(self):
        # Read the spot revision outside our own lock; SpotStore has its own.
        spots_rev = self.spots.rev()
        with self._lock:
            if self._meta is None:
                return json.dumps({'spots_rev': spots_rev}).encode()
            res, ox, oy, w, h = self._meta
            pose = None
            if self._pose is not None:
                pose = [round(self._pose[0], 3), round(self._pose[1], 3),
                        round(self._pose[2], 3)]
            # Path/goal go stale 15 s after the last /plan (goal reached or
            # aborted -> nothing republishes -> overlay disappears).
            plan, goal = None, None
            if self._plan and time.monotonic() - self._plan_time < 15.0:
                plan, goal = self._plan, list(self._goal)
            # Only the spot REVISION rides along here: /meta.json is polled at
            # 2 Hz, the spot list is fetched from /spots.json when it changes.
            return json.dumps({'res': res, 'ox': ox, 'oy': oy, 'w': w, 'h': h,
                               'upscale': UPSCALE, 'half': ROBOT_HALF,
                               'seq': self._seq, 'pose': pose,
                               'plan': plan, 'goal': goal,
                               'spots_rev': spots_rev}).encode()

    @staticmethod
    def _placeholder_png():
        pil = Image.new('RGB', (200, 60), (40, 40, 40))
        ImageDraw.Draw(pil).text((10, 25), 'waiting for /map...', fill=(255, 255, 255))
        buf = io.BytesIO()
        pil.save(buf, format='PNG')
        return buf.getvalue()


HTML = """<!doctype html><html><head><meta charset="utf-8">
<title>SLAM map</title>
<style>
 body{background:#222;color:#eee;font-family:sans-serif;text-align:center;margin:0;padding:12px}
 #wrap{position:relative;display:inline-block}
 img{image-rendering:pixelated;max-width:98vw;border:1px solid #444;cursor:crosshair;display:block}
 #ov{position:absolute;left:0;top:0;pointer-events:none}
 #bar{font-family:monospace;font-size:15px;margin:8px;min-height:1.2em}
 #goal{font-family:monospace;font-size:12px;text-align:left;white-space:pre-wrap;
       word-break:break-all;background:#111;border:1px solid #444;border-radius:4px;
       max-width:98vw;margin:8px auto;padding:8px;display:none}
 small{color:#999}
 #tools{margin:8px}
 #tools input[type=number]{width:5em;background:#111;color:#eee;border:1px solid #444;
       border-radius:3px;padding:2px 4px}
 #note{color:#fb0;font-family:monospace;font-size:13px;min-height:1.2em;margin:4px}
 table{border-collapse:collapse;margin:8px auto;max-width:98vw;font-size:14px}
 th,td{border:1px solid #444;padding:3px 7px;text-align:left}
 th{color:#999;font-weight:normal}
 td input{background:#111;color:#eee;border:1px solid #444;border-radius:3px;
       padding:2px 4px;width:10em}
 td.num{font-family:monospace;color:#bbb}
 #spotwrap{display:none}
</style></head><body>
<h3>Live SLAM map &mdash; robot in <span style="color:#f33">red</span>,
goal in <span style="color:#39f">blue</span>,
path in <span style="color:#3c6">green</span>,
parking spots in <span style="color:#fb0">amber</span></h3>
<div id="bar">hover over the map for coordinates</div>
<div id="wrap"><img id="m" src="/map.png"><canvas id="ov"></canvas></div>
<div id="tools">
  <button id="posebtn">&#128205; set robot pose</button>
  <button id="spotbtn">&#128658; add parking spot</button>
  &nbsp;cart <input id="clen" type="number" step="0.05" min="0.05" max="5">
  &times; <input id="cwid" type="number" step="0.05" min="0.05" max="5"> m
</div>
<div id="note"></div>
<pre id="goal"></pre>
<div id="spotwrap"><table id="spots"></table></div>
<small>click the map for a nav goal command &mdash; "set robot pose": click where the
robot IS, then a point it is FACING (publishes /initialpose, like RViz 2D Pose
Estimate) &mdash; "add parking spot": click where the cart sits, then a point it
FACES. Spots are saved next to the map; nothing here ever drives the robot.</small>
<script>
 let meta = null, seq = -1;
 const img = document.getElementById('m');
 const cv  = document.getElementById('ov');
 const bar = document.getElementById('bar');
 const goal = document.getElementById('goal');
 const note = document.getElementById('note');

 let pendingGoal = null;   // [x,y] clicked destination, shown until /plan takes over
 let spotsDoc = {spots: [], cart: {length: 0.8, width: 0.6}};
 let spotsRev = -1;
 let hoverPt = null;       // cursor in map meters, for the placement preview
 // Click-mode state. Declared up here because drawRobot() reads it and runs
 // before the parking-spot block further down.
 let poseMode = 0;         // 0=off, 1=await position, 2=await heading
 let poseXY = null;
 let spotMode = 0;         // same shape as poseMode; the two are exclusive
 let spotXY = null;
 let movingId = null;      // set when re-placing an existing spot

 function toPx(x, y, r){   // map meters -> display pixels
   const scale = r.width / img.naturalWidth;
   return [(x - meta.ox) / meta.res * meta.upscale * scale,
           (meta.h - (y - meta.oy) / meta.res) * meta.upscale * scale];
 }

 // Oriented rectangle: halfL along the heading, halfW across it. The robot and
 // the nav goal are squares, so footprint() just passes meta.half twice.
 function box(ctx, x, y, yaw, halfL, halfW, color, dashed, r, label){
   const pxPerM = meta.upscale / meta.res * (r.width / img.naturalWidth);
   const [px, py] = toPx(x, y, r);
   const hl = halfL * pxPerM, hw = halfW * pxPerM;
   ctx.save();
   ctx.translate(px, py);
   ctx.rotate(-yaw);                   // map yaw is CCW, canvas y points down
   ctx.strokeStyle = color; ctx.fillStyle = color; ctx.lineWidth = 2;
   ctx.setLineDash(dashed ? [6, 4] : []);
   ctx.strokeRect(-hl, -hw, 2*hl, 2*hw);
   ctx.beginPath(); ctx.moveTo(0, 0); ctx.lineTo(hl, 0); ctx.stroke();
   ctx.setLineDash([]);
   ctx.beginPath(); ctx.arc(0, 0, 3, 0, 7); ctx.fill();
   ctx.restore();
   if(label){                          // drawn after restore() so it stays upright
     // Clear the box's real vertical extent, which swaps as it rotates.
     const up = Math.abs(hl*Math.sin(yaw)) + Math.abs(hw*Math.cos(yaw));
     ctx.font = 'bold 12px sans-serif'; ctx.textAlign = 'center';
     ctx.lineWidth = 3; ctx.strokeStyle = '#222';
     ctx.strokeText(label, px, py - up - 6);   // dark halo: readable on any map
     ctx.fillStyle = color;
     ctx.fillText(label, px, py - up - 6);
     ctx.textAlign = 'start'; ctx.lineWidth = 2;
   }
 }

 function footprint(ctx, x, y, yaw, color, dashed, r){
   box(ctx, x, y, yaw, meta.half, meta.half, color, dashed, r, null);
 }

 function drawRobot(){
   const r = img.getBoundingClientRect();
   if(cv.width !== Math.round(r.width) || cv.height !== Math.round(r.height)){
     cv.width = Math.round(r.width); cv.height = Math.round(r.height);
   }
   const ctx = cv.getContext('2d');
   ctx.clearRect(0, 0, cv.width, cv.height);
   if(!meta || !meta.w || !img.naturalWidth) return;
   // Saved parking spots (amber), underneath everything else.
   ctx.globalAlpha = 0.8;
   spotsDoc.spots.forEach(s=>{
     const live = (s.id === movingId);
     box(ctx, s.x, s.y, s.yaw, s.length/2, s.width/2, '#fb0', live, r, s.name);
   });
   ctx.globalAlpha = 1;
   // Spot being placed: dashed preview that follows the cursor.
   if(spotMode && spotXY){
     const yaw = hoverPt ? Math.atan2(hoverPt[1]-spotXY[1], hoverPt[0]-spotXY[0]) : 0;
     box(ctx, spotXY[0], spotXY[1], yaw,
         cartLen()/2, cartWid()/2, '#ff0', true, r, null);
   }
   // Planned path (green line), from Nav2's /plan.
   if(meta.plan && meta.plan.length > 1){
     ctx.strokeStyle = '#3c6'; ctx.lineWidth = 2;
     ctx.beginPath();
     meta.plan.forEach((p, i)=>{
       const [px, py] = toPx(p[0], p[1], r);
       i ? ctx.lineTo(px, py) : ctx.moveTo(px, py);
     });
     ctx.stroke();
   }
   // Destination: blue footprint. Solid when Nav2 has an active plan; dashed
   // right after clicking, before the plan arrives.
   if(meta.goal){
     footprint(ctx, meta.goal[0], meta.goal[1], meta.goal[2], '#39f', false, r);
     pendingGoal = null;               // the real goal took over
   } else if(pendingGoal){
     footprint(ctx, pendingGoal[0], pendingGoal[1], 0, '#39f', true, r);
   }
   // Robot itself (red footprint), on top.
   if(meta.pose) footprint(ctx, meta.pose[0], meta.pose[1], meta.pose[2], '#f33', false, r);
 }

 async function tick(){
   try{ meta = await (await fetch('/meta.json')).json(); }catch(e){ return; }
   if(meta.seq !== undefined && meta.seq !== seq){
     seq = meta.seq;
     img.src = '/map.png?' + seq;      // reload image only when the map changed
   }
   if(meta.spots_rev !== undefined && meta.spots_rev !== spotsRev){
     await loadSpots();                // only when they actually changed
   }
   drawRobot();
 }
 setInterval(tick, __POLL__);
 tick();
 img.addEventListener('load', drawRobot);
 window.addEventListener('resize', drawRobot);

 function mapCoords(e){
   if(!meta || !meta.w) return null;
   const r = img.getBoundingClientRect();
   const nx = (e.clientX - r.left) / r.width  * img.naturalWidth;
   const ny = (e.clientY - r.top)  / r.height * img.naturalHeight;
   const x = meta.ox + nx / meta.upscale * meta.res;
   const y = meta.oy + (meta.h - ny / meta.upscale) * meta.res;
   return [x, y];
 }
 img.addEventListener('mousemove', e=>{
   const c = mapCoords(e);
   if(!c) return;
   hoverPt = c;
   if(spotMode && spotXY){ drawRobot(); return; }   // preview follows the cursor
   let t = `cursor  x: ${c[0].toFixed(2)} m   y: ${c[1].toFixed(2)} m`;
   if(meta && meta.pose) t += `   |   robot  x: ${meta.pose[0].toFixed(2)}  y: ${meta.pose[1].toFixed(2)}`;
   bar.textContent = t;
 });

 // ---- parking spots --------------------------------------------------
 const clen = document.getElementById('clen');
 const cwid = document.getElementById('cwid');
 const spotbtn = document.getElementById('spotbtn');
 const spotwrap = document.getElementById('spotwrap');
 const table = document.getElementById('spots');

 const cartLen = ()=> parseFloat(clen.value) || spotsDoc.cart.length;
 const cartWid = ()=> parseFloat(cwid.value) || spotsDoc.cart.width;

 async function post(url, body){
   const res = await fetch(url, {method: 'POST', body: JSON.stringify(body)});
   const out = await res.json().catch(()=>({error: 'bad response'}));
   if(!res.ok) throw new Error(out.error || res.status);
   return out;
 }

 function goalCmd(x, y, yaw){
   return `ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose ` +
     `"{pose: {header: {frame_id: map}, pose: {position: {x: ${x.toFixed(2)}, y: ${y.toFixed(2)}}, ` +
     `orientation: {z: ${Math.sin(yaw/2).toFixed(4)}, w: ${Math.cos(yaw/2).toFixed(4)}}}}}"`;
 }

 async function loadSpots(){
   try{ spotsDoc = await (await fetch('/spots.json')).json(); }catch(e){ return; }
   spotsRev = spotsDoc.rev;
   if(document.activeElement !== clen) clen.value = spotsDoc.cart.length;
   if(document.activeElement !== cwid) cwid.value = spotsDoc.cart.width;
   note.textContent = spotsDoc.note || '';
   renderSpots();
 }

 function renderSpots(){
   spotwrap.style.display = spotsDoc.spots.length ? 'block' : 'none';
   table.textContent = '';
   const head = table.insertRow();
   ['name', 'x', 'y', 'yaw', '', '', ''].forEach(h=>{
     const th = document.createElement('th'); th.textContent = h; head.appendChild(th);
   });
   spotsDoc.spots.forEach(s=>{
     const tr = table.insertRow();
     const nameCell = tr.insertCell();
     const input = document.createElement('input');
     input.value = s.name;                       // .value, never innerHTML
     input.addEventListener('change', async ()=>{
       try{ await post('/spots/update', {id: s.id, name: input.value}); await loadSpots(); }
       catch(err){ note.textContent = 'rename failed: ' + err.message; }
     });
     nameCell.appendChild(input);
     [s.x.toFixed(2), s.y.toFixed(2), (s.yaw*180/Math.PI).toFixed(0) + '°'].forEach(v=>{
       const td = tr.insertCell(); td.className = 'num'; td.textContent = v;
     });
     const go = document.createElement('button');
     go.textContent = 'goal command';
     go.title = 'shows the command -- does NOT drive the robot';
     go.addEventListener('click', ()=>{
       goal.style.display = 'block';
       goal.textContent = goalCmd(s.x, s.y, s.yaw);
       pendingGoal = null;
       bar.textContent = `command for "${s.name}" -- paste it in a terminal to drive there`;
     });
     tr.insertCell().appendChild(go);
     const mv = document.createElement('button');
     mv.textContent = 'move';
     mv.addEventListener('click', ()=>{ startSpotMode(s.id); });
     tr.insertCell().appendChild(mv);
     const del = document.createElement('button');
     del.textContent = '✕';
     del.addEventListener('click', async ()=>{
       if(!confirm(`Delete parking spot "${s.name}"?`)) return;
       try{ await post('/spots/delete', {id: s.id}); await loadSpots(); }
       catch(err){ note.textContent = 'delete failed: ' + err.message; }
     });
     tr.insertCell().appendChild(del);
   });
   drawRobot();
 }

 function setPoseMode(on){
   poseMode = on ? 1 : 0; poseXY = null;
   posebtn.style.background = on ? '#f33' : '';
 }
 function setSpotMode(on){
   spotMode = on ? 1 : 0; spotXY = null;
   if(!on) movingId = null;
   spotbtn.style.background = on ? '#fb0' : '';
   spotbtn.style.color = on ? '#000' : '';
 }
 function startSpotMode(id){            // the two modes are mutually exclusive
   setPoseMode(false);
   setSpotMode(true);
   movingId = id || null;
   bar.textContent = movingId ? 'click the cart’s NEW position'
                              : 'click where the cart sits';
   drawRobot();
 }

 spotbtn.addEventListener('click', ()=>{
   if(spotMode){ setSpotMode(false); bar.textContent = 'parking spot mode off'; drawRobot(); }
   else startSpotMode(null);
 });
 [clen, cwid].forEach(el=> el.addEventListener('change', async ()=>{
   try{ await post('/spots/cart', {length: cartLen(), width: cartWid()}); await loadSpots(); }
   catch(err){ note.textContent = 'cart size rejected: ' + err.message; }
 }));

 // ---- pose-set mode: click 1 = where the robot is, click 2 = a point it faces.
 const posebtn = document.getElementById('posebtn');
 posebtn.addEventListener('click', ()=>{
   const on = !poseMode;
   setSpotMode(false);
   setPoseMode(on);
   bar.textContent = on ? 'click where the robot IS' : 'pose mode off';
   drawRobot();
 });

 img.addEventListener('click', async e=>{
   const c = mapCoords(e);
   if(!c) return;
   if(poseMode === 1){
     poseXY = c;
     poseMode = 2;
     bar.textContent = `position (${c[0].toFixed(2)}, ${c[1].toFixed(2)}) — now click a point the robot is FACING`;
     return;
   }
   if(poseMode === 2){
     const yaw = Math.atan2(c[1]-poseXY[1], c[0]-poseXY[0]);
     try{
       await fetch(`/set_initialpose?x=${poseXY[0].toFixed(3)}&y=${poseXY[1].toFixed(3)}&yaw=${yaw.toFixed(3)}`);
       bar.textContent = `initial pose sent: x=${poseXY[0].toFixed(2)} y=${poseXY[1].toFixed(2)} yaw=${yaw.toFixed(2)} — watch the footprint snap`;
     }catch(err){ bar.textContent = 'failed to send initial pose'; }
     setPoseMode(false);
     return;
   }
   if(spotMode === 1){
     spotXY = c;
     spotMode = 2;
     bar.textContent = `cart at (${c[0].toFixed(2)}, ${c[1].toFixed(2)}) — now click a point it FACES`;
     drawRobot();
     return;
   }
   if(spotMode === 2){
     const yaw = Math.atan2(c[1]-spotXY[1], c[0]-spotXY[0]);
     const body = {x: spotXY[0], y: spotXY[1], yaw: yaw,
                   length: cartLen(), width: cartWid()};
     try{
       if(movingId){ await post('/spots/update', Object.assign({id: movingId}, body)); }
       else{ await post('/spots/add', body); }
       await loadSpots();
       bar.textContent = spotsDoc.saved
         ? `parking spot saved to ${spotsDoc.path}`
         : 'parking spot added but NOT saved to disk -- see the message above';
     }catch(err){ note.textContent = 'could not save the spot: ' + err.message; }
     setSpotMode(false);
     drawRobot();
     return;
   }
   goal.style.display = 'block';
   goal.textContent = goalCmd(c[0], c[1], 0);
   pendingGoal = c;                    // show the blue destination immediately
   drawRobot();
 });
 loadSpots();
</script></body></html>""".replace('__POLL__', str(MAP_POLL_MS))


def make_handler(node):
    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _send(self, body, ctype, code=200):
            self.send_response(code)
            self.send_header('Content-Type', ctype)
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _error(self, msg, code=400):
            self._send(json.dumps({'error': str(msg)}).encode(),
                       'application/json', code)

        def do_GET(self):
            if self.path.startswith('/map.png'):
                self._send(node.png(), 'image/png')
            elif self.path.startswith('/meta.json'):
                self._send(node.meta_json(), 'application/json')
            elif self.path.startswith('/spots.json'):
                self._send(node.spots.as_json(), 'application/json')
            elif self.path.startswith('/set_initialpose'):
                from urllib.parse import urlparse, parse_qs
                q = parse_qs(urlparse(self.path).query)
                try:
                    node.publish_initial_pose(
                        float(q['x'][0]), float(q['y'][0]), float(q['yaw'][0]))
                    self._send(b'ok', 'text/plain')
                except (KeyError, ValueError):
                    self._send(b'bad params', 'text/plain')
            else:
                self._send(HTML.encode(), 'text/html')

        # Spot edits carry a user-typed name and mutate a file on disk, so they
        # go over POST rather than following /set_initialpose's side-effecting
        # GET -- browsers are free to prefetch a GET.
        def do_POST(self):
            from urllib.parse import urlparse
            route = urlparse(self.path).path
            actions = {'/spots/add': node.spots.add,
                       '/spots/update': node.spots.update,
                       '/spots/delete': node.spots.delete,
                       '/spots/cart': node.spots.set_cart}
            action = actions.get(route)
            if action is None:
                self._error(f'unknown endpoint: {route}', 404)
                return
            try:
                length = int(self.headers.get('Content-Length') or 0)
            except ValueError:
                self._error('bad Content-Length')
                return
            if not 0 <= length <= MAX_BODY:
                self._error(f'body must be 0..{MAX_BODY} bytes')
                return
            try:
                req = json.loads(self.rfile.read(length) or b'{}')
                if not isinstance(req, dict):
                    raise ValueError('expected a JSON object')
                out = action(req)
            except (ValueError, KeyError, TypeError, UnicodeDecodeError) as e:
                self._error(e)
                return
            self._send(json.dumps(out).encode(), 'application/json')
    return H


def main():
    import argparse
    ap = argparse.ArgumentParser(description='Web map viewer (port 8000)')
    ap.add_argument('--map', help='saved map .yaml (from map_saver_cli) to preload')
    ap.add_argument('--spots',
                    help='cart parking spot JSON to read and write, normally '
                         '<map>.spots.json next to the map. Omit to disable '
                         'saving (spots can still be drawn, but are lost).')
    args, ros_argv = ap.parse_known_args()

    rclpy.init(args=ros_argv)
    node = MapRenderer(map_file=args.map, spots_path=args.spots)
    srv = ThreadingHTTPServer(('0.0.0.0', PORT), make_handler(node))
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        srv.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
