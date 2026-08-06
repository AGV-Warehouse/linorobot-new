# Under-Cart Docking — Phase 0–2 Checklist

The AGV docks by driving underneath a cart. Detection is **tagless**: a new
node (`cart_leg_detector.py`) finds the cart's four legs in the raw lidar
scan and feeds `detected_dock_pose` to the Nav2 docking server, which is
already configured with `use_external_detection_pose: true`
(`linorobot2_navigation/config/navigation.yaml`, `docking_server`).

Phases: **0** geometry gate (this checklist) → **1** leg detector →
**2** EKF odometry swap → *3 docking server bringup → 4 NFC cart identity →
5 carrying-the-cart (later)*.

---

## Phase 0 — Geometry gate (do this BEFORE trusting any software)

### 0.1 Tape-measure checklist

| # | Measurement | Pass criterion | Feeds |
|---|---|---|---|
| 1 | Cart under-deck clearance | ≥ robot max height (incl. lidar mast + cabling) + 30 mm | go/no-go |
| 2 | Leg spacing, lateral (inner edge ↔ inner edge) | ≥ 550 mm robot footprint + 2 × 50 mm margin | go/no-go |
| 3 | Leg spacing, shorter, center ↔ center | record | `rect_width` |
| 4 | Leg spacing, longer, center ↔ center | record | `rect_length` |
| 5 | Leg cross-section (width the lidar sees) & shape | record; flag if < 25 mm or chromed/shiny | `leg_diameter` |
| 5b | Measure 3–4 **at lidar scan height**, not the top — angled legs differ (practice table: 0.70 at top, 0.64 at scan height). Best source: the detector's own leg markers | record | `rect_*` |
| 6 | Lidar scan-plane height vs. leg | scan plane must hit **bare leg**, below any cross-brace/shelf | go/no-go |
| 7 | Is the leg rectangle square (length ≈ width)? | if yes, note it — pose has 90° ambiguity, detector needs the expected-pose seed | detector config |

Enter 3–5 into `linorobot2_navigation/config/cart_leg_detector.yaml`.

### 0.2 Scan-quality capture (existing stack only, no new code)

```bash
# terminal 1 — the usual stack
ros2 launch linorobot2_bringup robot.launch.py

# terminal 2 — teleop
python3 teleop_keyboard.py

# terminal 3 — record
ros2 bag record /scan_raw /scan /tf /tf_static /odom_rf2o /odom /odom/unfiltered /imu/data
```

Drive: approach from ~2 m → slow entry under the cart → hold centered ~10 s
→ exit. Repeat 3×, at least one run entering at a deliberately bad angle.

Inspect in RViz (`/scan_raw`, fixed frame `odom`):

- [ ] At 0.5–2 m, all four legs show as distinct clusters of ≥ 3 points.
- [ ] While underneath, the legs around the robot are still visible in `/scan_raw`.

### 0.3 Gate decision

- **Pass** → measurements become the detector config; the bags become the
  Phase 1 offline test fixture.
- **Sparse/flickering leg clusters** → wrap legs in matte reflective tape and
  re-record. Still bad → fall back to the camera + belly-AprilTag design
  (only the detector node changes; Phases 2–3 carry over unchanged).
- **Clearance or leg-gap failure** → hardware change (lidar remount / cart
  selection) before writing any software.

---

## Phase 1 — Leg detector validation

Run standalone (works against live scans or a Phase 0 bag):

```bash
ros2 launch linorobot2_navigation cart_leg_detector.launch.py
# offline: ros2 bag play <phase0_bag> in another terminal
```

Watch `cart_leg_detector/markers` in RViz: orange spheres = leg candidates,
green rectangle = accepted fit, blue arrow = filtered `detected_dock_pose`.

For offline bag exploration set `gate_radius: 0.0` (accept anything); for
any live use keep gating on and seed `expected_cart_*` — gating is the guard
against look-alike leg patterns (racking, chairs, a second cart).

Naming note: the detector's own knobs use "cart" (`expected_cart_*`), but
the output topic stays `detected_dock_pose` — that name is hardcoded in
Nav2's `SimpleChargingDock` plugin and cannot be renamed on our side.

Acceptance:

- [ ] Detection on ≥ 95 % of scans within 2 m of the cart.
- [ ] Position jitter σ ≤ 1 cm on the hold-centered bag segment.
- [ ] No orientation flips during entry (the 2–3-visible-legs case).
- [ ] Zero detections on a cart-free bag segment (with gating configured).

## Phase 2 — Odometry swap validation (rf2o → EKF)

The EKF (`robot_localization`, config `linorobot2_base/config/ekf.yaml`) now
owns `odom->base_footprint`, fusing Teensy wheel odometry
(`odom/unfiltered`) with the IMU. rf2o still publishes `/odom_rf2o`
(topic only) for comparison; disable with `rf2o:=false`.

Preflight:

- [ ] `ros2 topic info /imu/data --verbose` → exactly **one** publisher (the
  Jetson `mpu6050_imu` node). The Teensy firmware also publishes `imu/data`
  if it finds an MPU6050 on its own bus — two publishers corrupts the EKF.
- [ ] Push the robot ~1 m by hand **with the micro-ROS agent running**
  (AK10-9s back-drive, encoders still count): `ros2 topic echo /odom/unfiltered`
  tracks direction and roughly the distance.

Regression:

- [ ] `ros2 run tf2_tools view_frames` → `odom->base_footprint` published
  only by `ekf_filter_node`; `map->odom` by slam_toolbox (or AMCL in nav mode).
- [ ] 2 m straight line: `/odom` x error < 2 % (tape measure).
- [ ] 2 × 360° spin in place: yaw error < 5°.
- [ ] Re-map a known area: map quality ≥ the existing baseline map.
- [ ] **The money test**: teleop under the cart while echoing `/odom` — pose
  stays smooth where rf2o (`/odom_rf2o`, still recorded) glitches.

Known caveat: with `micro_ros:=false` there is no wheel odometry, the EKF
has no input, and the odom TF goes stale — keep the agent running even for
hand-push sessions.

## Phase 3 — First docking attempt (practice table)

The docking server starts with the nav stack (`navigation.launch.py` →
nav2_bringup) and already consumes `detected_dock_pose`
(`use_external_detection_pose: true`). The external-detection offsets in
`navigation.yaml` are zeroed for the leg detector — the upstream `-1.57`
roll/pitch example values are AprilTag corrections and MUST stay 0 here.

Stack (4 terminals):

```bash
ros2 launch linorobot2_bringup robot.launch.py slam:=false   # 1: bringup, nav mode
ros2 launch linorobot2_navigation navigation.launch.py      # 2: Nav2 (+ docking server)
# set the AMCL pose within ~30 s (map page :8000) — pose_keeper may restore it
ros2 launch linorobot2_navigation cart_leg_detector.launch.py   # 3: detector
```

Preflight (terminal 4):

- [ ] `ros2 topic echo /detected_dock_pose --once` → stable pose at the cart
  center (this is the Phase 1/2 static test).
- [ ] `ros2 lifecycle get /docking_server` → `active`.
- [ ] Robot pre-positioned by teleop ~1 m in front of the cart, roughly
  facing it (first attempts skip staging navigation). Floor clear, cart
  clearance verified (Phase 0).
- [ ] The entry axis is chosen when the detector first locks: it aims from
  the robot through the cart center, so park **square-on to the side you
  want to enter** (the long side has the bigger leg gap) BEFORE starting
  the detector. If you reposition to a different side afterwards, restart
  the detector (terminal 3) to re-lock the axis.

Send the goal, filling x/y/z/w from the echoed `detected_dock_pose`
(keep `--feedback`; Ctrl-C on this command CANCELS the goal and stops the
approach — that is the abort button):

```bash
ros2 action send_goal /dock_robot nav2_msgs/action/DockRobot \
  "{use_dock_id: false, dock_type: simple_charging_dock, navigate_to_staging_pose: false, \
    dock_pose: {header: {frame_id: odom}, pose: {position: {x: X, y: Y}, orientation: {z: Z, w: W}}}}" \
  --feedback
```

The server drives to the staging point (`staging_x_offset: -0.7` → 0.7 m in
front of the cart center), then creeps forward, continuously refining
against `detected_dock_pose`, until the base sits within `docking_threshold`
(5 cm) of the cart center. No charger contact exists, so success == pose
(battery/stall checks are off).

Back out:

```bash
ros2 action send_goal /undock_robot nav2_msgs/action/UndockRobot \
  "{dock_type: simple_charging_dock}"
```

Acceptance:

- [ ] Approach is smooth (no oscillation at the 0.05 m/s crawl); robot stops
  under the cart center within ~5 cm laterally.
- [ ] Detector holds lock during the whole approach (echo keeps streaming —
  the entry-occlusion case).
- [ ] Ctrl-C mid-approach stops the robot promptly (cancel path works).
- [ ] Undock returns it cleanly ~0.7 m out the way it came.

Once this passes, enable staging navigation (`navigate_to_staging_pose:
true`) so Nav2 plans the drive to the staging point from anywhere on the map.
