# HARDWARE — what this robot physically is

Single source of truth for boards, wiring, ports, and physical quirks.
Update this file whenever wiring or hardware changes.

## Overview

2WD differential-drive base:

| Part | Detail |
|------|--------|
| Compute | NVIDIA Jetson (this machine, Ubuntu / Linux tegra) |
| MCU | **Nucleo-H753ZI** (replacing the Teensy 4.1, 2026-10). Same `firmware/`, one PlatformIO env per board: `nucleo_h753zi` / `teensy41` |
| Motors | 2× CubeMars **AK10-9**, MIT mode over CAN @ 1 Mbps |
| Lidar | RPLIDAR on `/dev/ttyUSB0`, full 360° minus battery-box mask |
| IMU | MPU6050 (I2C on the MCU) |

## Serial / ports

- `/dev/ttyACM0` — MCU micro-ROS serial, **baud 921600** (set in
  `firmware/src/main.cpp`). Exactly one micro-ROS agent may hold it.
  Teensy: USB CDC (baud nominal). Nucleo: USART3 (PD8/PD9) → the ST-LINK
  virtual COM port on CN1 — a real UART, so firmware and agent baud must
  match. The port survives MCU resets and flashes.
- `/dev/ttyUSB0` — RPLIDAR.
- Port **8000** — live map web page served by `robot.launch.py`
  (`http://<robot-ip>:8000`, or `ssh -L 8000:localhost:8000`). Used to set
  the AMCL initial pose and click nav goals.

## CAN bus (Teensy 4.1)

- CAN2: Teensy pin 0 = CRX2, pin 1 = CTX2 → 3.3 V transceiver
  (SN65HVD230 / TJA1051T/3) → motors, 1 Mbps.
- Transceiver mode pin MUST be tied low/enabled: SN65HVD230 **RS ≤ 1 kΩ to
  GND**; TJA1051 **S pin LOW**. Floating = silent mode = receive-only —
  this exact fault has eaten days.
- Motor CAN IDs: left = controller `0x68` (cmd `0x868`),
  right = `0x69` (cmd `0x869`).
- Link health is exposed live on `/motor_current`
  (see TROUBLESHOOTING.md → "CAN link health").

## CAN bus (Nucleo-H753ZI)

FDCAN1, wiring per UM2407 Table 20 — straight-through, never crossed:

| Transceiver | Nucleo | Header |
|---|---|---|
| RXD / CRX | **PD0** (D67, CAN_RX) | CN9 pin 25 |
| TXD / CTX | **PD1** (D66, CAN_TX) | CN9 pin 27 |
| VCC (SN65HVD230) | +3V3 | CN8 pin 7 |
| VCC / VIO (TJA1051T/3) | +5V / +3V3 | CN8 pin 9 / CN8 pin 7 |
| GND, RS/S mode pin | GND | CN8 pin 11 or 13 |

- CN9 odd column, bottom end: PF0 (21), **GND (23)**, PD0 (25), PD1 (27),
  PG0 (29, last pin). Pin 23 is the meter landmark: it beeps to GND, and the
  next two pins are RX then TX. (An earlier version of this file said pin 23
  was PF0 — wrong; the CAN pins were right.)
- Same transceiver rules as the Teensy (mode pin low, 3.3 V logic); CANH/CANL
  and termination unchanged (≈ 60 Ω across CANH–CANL, powered off).
- Firmware: `firmware/include/fdcan_h7.h`. Kernel clock = HSE 8 MHz (ST-LINK
  MCO, bypass), NBTP = 0x500 (exact 1 Mbps, 87.5 % sample point), PD0/PD1 AF9
  push-pull — the values verified on this board during the first bring-up.
- Bus-off: the M_CAN latches CCCR.INIT and never leaves it by itself. The
  firmware clears it on its next CAN poll (FlexCAN on the Teensy did this
  automatically), so a fixed CAN fault recovers without a reset.
- Live-debug register recipes (openocd + ST-LINK, FDCAN TEST register,
  listen-only pokes) are in TROUBLESHOOTING.md → "Nucleo FDCAN".
- MPU6050 (optional): SDA = PB9 (CN7 pin 4), SCL = PB8 (CN7 pin 2), 3.3 V.
- History: the first Nucleo port lived on `claude/teensy-nucleo-migration-b9g26c`,
  which was never pushed and is gone. The current port was redone from `main`
  in 2026-10.

## Motors / firmware behavior

- Boot: firmware sends the MIT "enter motor mode" handshake to both motors
  **unconditionally**, braked immediately. A small boot twitch is expected.
  (Probe-and-skip was tried and is unsafe: AK10-9s stream status even when
  NOT in MIT mode.)
- Motor battery power-cycled while the MCU stays up → motors resume
  streaming on their own but may not be in MIT mode; **reboot the MCU**.
- Overcurrent latch: > `OVERCURRENT_AMPS` (8 A) for `OVERCURRENT_MS` (1 s)
  brakes both motors until a zero cmd_vel releases it (teleop space bar).
- Calibrated constants in `firmware/include/config.h` (hardware
  measurements — recalibrate, don't look up):
  - `ERPM_TO_WHEEL_RADPS = 0.01237` (tape test 2026-07-20, TESTING.md §4)
  - `WHEEL_DIAMETER`, `LR_WHEELS_DISTANCE` — measured on the robot
  - `LEFT_MOTOR_DIR = +1`, `RIGHT_MOTOR_DIR = -1`

## Physical blind zones & footprint facts

- The **battery box sits behind the robot center** and is masked out of
  `/scan` by `linorobot2_bringup/config/box_laser_filter.yaml`
  (base_footprint frame: x −1.0..−0.10 m, y ±0.5 m). Everything inside the
  box is a permanent lidar blind spot — this is why the nav recovery BT
  uses a short 0.15 m BackUp instead of the stock 0.30 m.
- Collision monitor footprint: circumscribed radius ≈ 0.39 m. Any scan
  return inside it (robot-mounted part not yet masked, or a real wall)
  zeroes ALL cmd_vel — see TROUBLESHOOTING.md.

## Flashing

- Teensy: `cd firmware && pio run -e teensy41 -t upload`. First attempt
  often fails (`error writing to Teensy`) — retry; confirm `[SUCCESS]` or
  the Teensy may be stranded in its bootloader (no `/dev/ttyACM0`).
- Flashing makes the USB device vanish, which **kills the micro-ROS agent
  and it does not respawn** — restart `robot.launch.py` after a flash.
- Nucleo: `cd firmware && pio run -e nucleo_h753zi -t upload` over the
  ST-LINK USB (CN1). The virtual COM port stays up, so the agent reconnects
  in ~2 s on its own. `reset run` via openocd (or the black B2 button)
  reboots the firmware. Needs PlatformIO's udev rules for the ST-LINK
  (firmware/README.md). Fallback: copy `.pio/build/nucleo_h753zi/firmware.bin`
  onto the `NOD_H753ZI` USB drive.
- Either board: the MCU stops sending CAN while it flashes, and the motors
  keep executing their last MIT command — flash with the robot stopped.
