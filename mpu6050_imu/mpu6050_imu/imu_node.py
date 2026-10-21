#!/usr/bin/env python3
# MPU6050 driver for the Jetson's I2C bus -> sensor_msgs/Imu on imu/data.
#
# The EKF (linorobot2_base/config/ekf.yaml) fuses ONLY angular_velocity.z from
# this topic, so the priority is a clean, bias-free yaw rate. No orientation is
# estimated (orientation_covariance[0] = -1 per REP-145).
#
# The gyro bias is measured at startup: keep the robot STILL for the first
# ~2 s after launch (calibration_time). Motion during calibration is detected
# and logged as a warning, since the resulting bias would bias the EKF yaw.
#
# Axes: sensor axes are published as-is in frame_id (imu_link). On this robot
# the chip reads +1 g on z at rest, i.e. z is up, so +gyro z = CCW from above,
# matching REP-103 with the identity base_footprint->imu_link transform.
#
# No smbus dependency: talks to /dev/i2c-N directly via the I2C_RDWR ioctl
# (repeated-start register reads, so one 14-byte burst is a coherent sample).

import ctypes
import fcntl
import math
import os
import statistics
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

# linux/i2c-dev.h, linux/i2c.h
I2C_SLAVE = 0x0703
I2C_RDWR = 0x0707
I2C_M_RD = 0x0001

# MPU6050 registers
REG_SMPLRT_DIV = 0x19
REG_CONFIG = 0x1A
REG_GYRO_CONFIG = 0x1B
REG_ACCEL_CONFIG = 0x1C
REG_ACCEL_XOUT_H = 0x3B
REG_PWR_MGMT_1 = 0x6B
REG_WHO_AM_I = 0x75

GRAVITY = 9.80665
# FS_SEL / AFS_SEL code -> LSB per unit
GYRO_RANGES = {250: (0, 131.0), 500: (1, 65.5), 1000: (2, 32.8), 2000: (3, 16.4)}
ACCEL_RANGES = {2: (0, 16384.0), 4: (1, 8192.0), 8: (2, 4096.0), 16: (3, 2048.0)}


class _I2cMsg(ctypes.Structure):
    _fields_ = [('addr', ctypes.c_uint16), ('flags', ctypes.c_uint16),
                ('len', ctypes.c_uint16), ('buf', ctypes.POINTER(ctypes.c_uint8))]


class _I2cRdwrData(ctypes.Structure):
    _fields_ = [('msgs', ctypes.POINTER(_I2cMsg)), ('nmsgs', ctypes.c_uint32)]


_libc = ctypes.CDLL(None, use_errno=True)


class I2cDevice:
    def __init__(self, bus, addr):
        self.addr = addr
        self.fd = os.open(f'/dev/i2c-{bus}', os.O_RDWR)
        fcntl.ioctl(self.fd, I2C_SLAVE, addr)

    def close(self):
        os.close(self.fd)

    def write_reg(self, reg, value):
        os.write(self.fd, bytes([reg, value & 0xFF]))

    def read_regs(self, reg, n):
        wbuf = (ctypes.c_uint8 * 1)(reg)
        rbuf = (ctypes.c_uint8 * n)()
        msgs = (_I2cMsg * 2)(
            _I2cMsg(self.addr, 0, 1, ctypes.cast(wbuf, ctypes.POINTER(ctypes.c_uint8))),
            _I2cMsg(self.addr, I2C_M_RD, n, ctypes.cast(rbuf, ctypes.POINTER(ctypes.c_uint8))),
        )
        data = _I2cRdwrData(ctypes.cast(msgs, ctypes.POINTER(_I2cMsg)), 2)
        if _libc.ioctl(self.fd, I2C_RDWR, ctypes.byref(data)) < 0:
            err = ctypes.get_errno()
            raise OSError(err, os.strerror(err))
        return bytes(rbuf)


def _s16(hi, lo):
    v = (hi << 8) | lo
    return v - 65536 if v > 32767 else v


class Mpu6050Node(Node):
    def __init__(self):
        super().__init__('mpu6050_imu')
        self.bus = self.declare_parameter('i2c_bus', 7).value
        self.addr = self.declare_parameter('i2c_addr', 0x68).value
        self.frame_id = self.declare_parameter('frame_id', 'imu_link').value
        rate_hz = self.declare_parameter('rate_hz', 100.0).value
        gyro_range = self.declare_parameter('gyro_range_dps', 500).value
        accel_range = self.declare_parameter('accel_range_g', 2).value
        self.calib_time = self.declare_parameter('calibration_time', 2.0).value
        # Std-dev of gyro samples during calibration above this => robot moved.
        self.calib_motion_dps = self.declare_parameter('calibration_motion_dps', 0.5).value
        gyro_std = self.declare_parameter('gyro_noise_std', 0.01).value     # rad/s
        accel_std = self.declare_parameter('accel_noise_std', 0.05).value   # m/s^2
        if self.declare_parameter('publish_odom_tf', False).value:
            self.get_logger().warn(
                'publish_odom_tf:=true is not supported by this driver; no TF is '
                'published (the EKF owns odom->base_footprint).')

        if gyro_range not in GYRO_RANGES or accel_range not in ACCEL_RANGES:
            raise ValueError(f'gyro_range_dps must be one of {list(GYRO_RANGES)}, '
                             f'accel_range_g one of {list(ACCEL_RANGES)}')
        self.gyro_code, self.gyro_lsb = GYRO_RANGES[gyro_range]
        self.accel_code, self.accel_lsb = ACCEL_RANGES[accel_range]
        self.gyro_cov = gyro_std ** 2
        self.accel_cov = accel_std ** 2

        self.dev = None
        self.gyro_bias = (0.0, 0.0, 0.0)  # rad/s
        self.consecutive_errors = 0

        self._open_and_configure()
        self._calibrate_gyro()

        self.pub = self.create_publisher(Imu, 'imu/data', 10)
        self.timer = self.create_timer(1.0 / rate_hz, self._tick)
        self.get_logger().info(
            f'MPU6050 on /dev/i2c-{self.bus} @ 0x{self.addr:02x}: publishing imu/data '
            f'at {rate_hz:.0f} Hz in frame {self.frame_id} '
            f'(gyro +/-{gyro_range} dps, accel +/-{accel_range} g)')

    def _open_and_configure(self):
        if self.dev is not None:
            try:
                self.dev.close()
            except OSError:
                pass
        self.dev = I2cDevice(self.bus, self.addr)
        who = self.dev.read_regs(REG_WHO_AM_I, 1)[0]
        if who != 0x68:
            self.get_logger().warn(
                f'WHO_AM_I = 0x{who:02x} (expected 0x68); continuing, may be a clone')
        self.dev.write_reg(REG_PWR_MGMT_1, 0x80)  # device reset
        time.sleep(0.1)
        self.dev.write_reg(REG_PWR_MGMT_1, 0x01)  # wake, clock = gyro X PLL
        time.sleep(0.05)
        self.dev.write_reg(REG_CONFIG, 0x03)      # DLPF ~44 Hz, 1 kHz internal rate
        self.dev.write_reg(REG_SMPLRT_DIV, 9)     # 1 kHz / (1 + 9) = 100 Hz
        self.dev.write_reg(REG_GYRO_CONFIG, self.gyro_code << 3)
        self.dev.write_reg(REG_ACCEL_CONFIG, self.accel_code << 3)
        time.sleep(0.1)  # let the DLPF settle

    def _read_raw(self):
        """Return (ax, ay, az) in m/s^2 and (gx, gy, gz) in rad/s, bias NOT removed."""
        b = self.dev.read_regs(REG_ACCEL_XOUT_H, 14)
        ax, ay, az = (_s16(b[i], b[i + 1]) / self.accel_lsb * GRAVITY for i in (0, 2, 4))
        # b[6:8] is temperature
        gx, gy, gz = (math.radians(_s16(b[i], b[i + 1]) / self.gyro_lsb) for i in (8, 10, 12))
        return (ax, ay, az), (gx, gy, gz)

    def _calibrate_gyro(self):
        self.get_logger().info(
            f'Calibrating gyro bias for {self.calib_time:.1f} s -- keep the robot still')
        samples = []
        end = time.monotonic() + self.calib_time
        while time.monotonic() < end:
            try:
                samples.append(self._read_raw()[1])
            except OSError as e:
                self.get_logger().warn(f'I2C read failed during calibration: {e}')
            time.sleep(0.01)
        if len(samples) < 20:
            self.get_logger().error(
                f'Only {len(samples)} calibration samples; publishing with zero gyro bias')
            return
        axes = list(zip(*samples))
        self.gyro_bias = tuple(statistics.fmean(a) for a in axes)
        worst_std_dps = max(math.degrees(statistics.pstdev(a)) for a in axes)
        bias_dps = ', '.join(f'{math.degrees(v):+.2f}' for v in self.gyro_bias)
        self.get_logger().info(
            f'Gyro bias [{bias_dps}] dps from {len(samples)} samples '
            f'(noise std {worst_std_dps:.2f} dps)')
        if worst_std_dps > self.calib_motion_dps:
            self.get_logger().warn(
                f'Gyro noise during calibration {worst_std_dps:.2f} dps > '
                f'{self.calib_motion_dps} dps: the robot was probably moving. Yaw rate '
                'will be biased -- restart this node with the robot still.')

    def _tick(self):
        try:
            (ax, ay, az), (gx, gy, gz) = self._read_raw()
        except OSError as e:
            self.consecutive_errors += 1
            self.get_logger().warn(f'I2C read failed: {e}', throttle_duration_sec=2.0)
            if self.consecutive_errors % 50 == 0:
                # The chip may have browned out / reset (it then wakes up asleep).
                try:
                    self._open_and_configure()
                    self.get_logger().warn('Re-initialized MPU6050 after repeated I2C errors')
                except OSError as e2:
                    self.get_logger().error(f'Re-init failed: {e2}', throttle_duration_sec=5.0)
            return
        self.consecutive_errors = 0

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.orientation_covariance[0] = -1.0  # orientation not provided
        msg.angular_velocity.x = gx - self.gyro_bias[0]
        msg.angular_velocity.y = gy - self.gyro_bias[1]
        msg.angular_velocity.z = gz - self.gyro_bias[2]
        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az
        for i in (0, 4, 8):
            msg.angular_velocity_covariance[i] = self.gyro_cov
            msg.linear_acceleration_covariance[i] = self.accel_cov
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Mpu6050Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
