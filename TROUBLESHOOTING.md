# TROUBLESHOOTING — symptom-first playbook

Every failure mode below was hit on real hardware and diagnosed the hard
way. Check here before re-deriving a diagnosis. Wiring/board facts live in
HARDWARE.md; per-change pass criteria live in TESTING.md.

## "Robot won't move" — check in THIS order

1. **Stacked / dead micro-ROS agents** (cheapest, most common):
   ```bash
   fuser -v /dev/ttyACM0        # must show exactly ONE holder
   ps aux | grep micro_ros_agent | grep -v grep
   ```
   Killing the `ros2 run micro_ros_agent ...` wrapper does NOT kill the
   agent binary — repeated test runs stack agents that each steal a share
   of serial RX bytes, mimicking a broken board (sessions taking ~20 s,
   dropping every ~2 s, topics flickering, cmd_vel ignored/delayed).
   Kill strays with `pkill -f micro_ros_agent` (exit 144 = it matched your
   own shell, not a failure). The agent also does NOT respawn if it dies
   (e.g. USB vanished during a flash): `/odom/unfiltered` silent while
   `/dev/ttyACM0` exists → restart the launch.

2. **CAN link health** — `/motor_current` carries diagnostics:
   ```bash
   ros2 topic echo /motor_current --once --field data
   # [0] left A  [1] right A  [2] frames rx since boot
   # [3] last raw CAN id      [4] TX err counter  [5] RX err counter
   ```
   Healthy: `[4]`=`[5]`=0, `[2]` climbing. `[4]` bouncing high with
   `[5]`=0 = TX wiring fault: commands die between MCU and bus while
   odometry still streams (receive path fine). With motor battery OFF
   check: TX wire to transceiver TXD, transceiver mode pin (must be
   low/enabled — floating = silent = RX-only!), VCC/GND, CANH/CANL +
   termination. CAN auto-retry masks a sick link — nonzero `[4]` deserves
   attention even when driving "works".

3. **Motor battery was power-cycled** while the MCU stayed up → motors
   stream feedback but aren't in MIT mode; they ignore every command.
   Reboot the MCU (reset button or re-flash).

4. **Overcurrent latch engaged**: watch `[0]`/`[1]` on `/motor_current`;
   after ~1 s above 8 A both motors brake and ignore cmd_vel until a zero
   command (teleop space) releases the latch.

5. **Autonomous only (teleop works)** → next section.

## Nav2 sends goals but `/cmd_vel` is silent

Root-caused 2026-07-19. Verify the chain stage by stage — record
`/cmd_vel_nav`, `/cmd_vel_smoothed`, `/cmd_vel`, `/odom`, `/rosout` while
sending a goal; per-stage message counts pinpoint the zeroing layer.

- **Collision monitor is the usual suspect.** FootprintApproach treats ANY
  scan return inside the ~0.39 m circumscribed footprint as collision at
  t=0 and scales every twist (forward, reverse, spin) to zero — it also
  explains failed Spin/BackUp recoveries. After `stop_pub_timeout` (2 s)
  it stops publishing entirely: **total silence on `/cmd_vel` is its
  permanent-stop signature.** `/collision_monitor_state` publishes only on
  state CHANGE, so echo may show nothing.
  Fix depends on the offending points: robot-mounted part → extend the
  mask in `box_laser_filter.yaml`; real obstacle → move the robot.
- **Startup deadlock (fixed, don't reintroduce):** controller accel clamp
  ≈ max speed + smoother deadband > first commanded step → robot never
  moves → odom stays 0 → clamp never opens. Keep controller accel limits
  well above the 0.05 m/s cap and deadband at 0.
- **Progress checker:** required_movement must be ≪ (max speed × timeout),
  or "Failed to make progress" loops are guaranteed. Current: 0.1 m / 15 s.
- **AMCL pose silently invalid:** hand-moving/lifting the robot produces no
  odom change — map walls shift ~0.5 m and planning goes insane. Re-set the
  pose via the map page after ANY manual move; `pose_keeper.py` restores
  the last pose only on startup.
- Stock BT BackUp (0.30 m) reverses into the battery-box blind zone —
  that's why `navigate_to_pose_doorway_recovery.xml` exists (0.15 m,
  recovery order: clear costmaps → short backup → wait → spin).

## ros2 CLI pitfalls on this Jetson

- `ros2 topic echo/hz` piped to a file buffers — killed by `timeout` it
  leaves an EMPTY file. Prefix with `PYTHONUNBUFFERED=1`.
- DDS discovery under load takes 2–3 s: `ros2 topic pub` injection tests
  must publish **≥ 10 s** or the subscriber never matches.
- `ros2 topic echo /scan` truncates `ranges` — use `--full-length`.
- The Jetson CPU is the budget: rf2o alone burns ~50% of a core; planner
  and controller rates in `navigation.yaml` were lowered deliberately.
- `grep -c "Node name"` on `ros2 topic info -v` also counts the
  `Node namespace` lines (double the endpoints). Use `"Node name:"`.
- `ros2 topic pub` to a TRANSIENT_LOCAL subscriber (`/object_boxes`) needs
  `--qos-durability transient_local`, or it never matches and never sends.
- `pkill -f <pattern>` run from a script/`bash -c` whose own command line
  contains the pattern kills that shell too. Kill launches by PID.

## Nucleo FDCAN bring-up

Wiring truth in HARDWARE.md. Debug techniques proven on this board
(PlatformIO-bundled openocd + ST-LINK, works while firmware runs):

```bash
~/.platformio/packages/tool-openocd/bin/openocd -s ~/.platformio/packages/tool-openocd/openocd/scripts \
  -f interface/stlink.cfg -f target/stm32h7x.cfg -c init -c "mdw <addr>" -c shutdown
```

- FDCAN1 base `0x4000A000`: CCCR 0x18, ECR 0x40, PSR 0x44, RXF0S 0xA4,
  TXFQS 0xC4, TXBRP 0xCC, NBTP 0x1C, TEST 0x10. GPIOD base `0x58020C00`
  (MODER/PUPDR/IDR/BSRR at 0x00/0x0C/0x10/0x18).
- **Listen-only poke** (watch bus without bus-off risk): CCCR INIT+CCE →
  set MON (bit 5) → clear INIT.
- **Force-dominant TX-pin test**: CCCR=0x3 → 0x83 → TEST TX[6:5]=0b10
  drives the real TX pin through the AF path — verifies FDCAN→pin without
  firmware. TEST.RX (bit 7) is NOT trustworthy while INIT=1.
- **Internal loopback (TEST.LBCK) ignores the physical RX pin** — it tests
  peripheral + message RAM only; never use it to "prove" the analog loop.
- **Floating-pin detection**: enable internal pull-down via PUPDR; a driven
  push-pull line holds high, a floating pin collapses. When loopback-testing
  adjacent pins, park a pull-up on the observed pin — a floating no-pull
  input follows its neighbor capacitively and fakes a pass.
- **Error decoding**: Bit0 error (LEC=5) + TEC near 248/bus-off can ONLY be
  the local TX path (other nodes can't turn dominant into recessive).
  No-ACK gives LEC=3 and caps at TEC 128 (error passive), never bus-off.
- **Bit-rate downshift** (NBTP NBRP=7 → 125 kbps): if Bit0 persists at
  125 k, rule out slew/loop-delay entirely.
- Bus-off: the firmware clears CCCR.INIT on its next CAN poll, so the link
  recovers on its own once the fault is fixed. If `/motor_current` `[4]`
  keeps climbing back to bus-off, the TX path is still broken.
- Serial symptoms that mimic all of this: stacked agents (section 1).

## Session notes 2026-11-11 (Nucleo + object-boxes bring-up)

Each of these cost real time; check them first next time.

- **Lidar dies at launch: `Error, unexpected error, code: 80008004`.**
  `robot.launch.py` defaults `lidar_port:=/dev/rplidar`, a udev symlink that
  does not exist on this Jetson. The RPLIDAR (CP2102) is `/dev/ttyUSB0`:
  launch with `lidar_port:=/dev/ttyUSB0` until a udev rule is added.
- **Nucleo silent: micro-ROS agent runs, no session, no `/odom/unfiltered`,
  `/cmd_vel` has 0 subscribers.** Diagnosed without touching the board:
  agent at `-v6` received zero bytes; OpenOCD over the ST-LINK showed 3.26 V,
  core running, no fault bits; DWT PC sampling put the firmware in
  `Uart::read()` (waiting for the agent's ping reply). So the firmware was
  fine and the ST-LINK USB serial bridge was wedged. Fix: **unplug/replug
  the Nucleo's USB (CN1)**, then restart the agent. Resetting or
  power-cycling the MCU alone does not re-enumerate `/dev/ttyACM0`.
  Read-only probe used:
  `~/.platformio/packages/tool-openocd/bin/openocd -s ~/.platformio/packages/tool-openocd/openocd/scripts -f interface/stlink.cfg -f target/stm32h7x.cfg -c init -c "mdw 0xE000EDF0" -c shutdown`
- **Nav2 navigation never activates** (`Failed to activate local_costmap
  because transform from base_footprint to odom did not become available`,
  `Aborting bringup`): navigation was started while the base had no odom.
  Once `/odom/unfiltered` flows, restart `navigation.launch.py`; it does
  not retry by itself.
- **AGV app can't connect, but SSH from the same Mac works.** rosbridge on
  :9090 was fine (handshake OK from the Jetson, no firewall, `curl` from the
  Mac reached :8000). The browser was blocked: Chrome failed even with
  macOS Privacy > Local Network switched on (it had 7 stale Chrome entries);
  **Arc worked immediately**. Test with `curl http://<robot-ip>:8000` from the
  Mac: if curl works and the browser doesn't, it's the browser. Workaround:
  `ssh -L 9090:localhost:9090 -L 8000:localhost:8000 <user>@<robot-ip>` and
  `?ros=ws://localhost:9090`.
- **Robot "stuck in a corner", every goal fails** (`GridBased plugin failed
  to plan ... Failed to create plan with tolerance`, BackUp recovery, then
  cancel). The robot marker sat on a mapped wall: AMCL was mislocalized
  (pose restored by `pose_keeper` was stale), so the start cell is in the
  wall's inflation and no plan exists. `python3 scan_map_fit.py` scored 34%
  of scan points on map walls. Fix: **Set Pose** at the true position, check
  the score rises, then resend the goal. Do not tune the planner for this.
