#!/usr/bin/env python3
import threading
import time
import math
import serial

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from geometry_msgs.msg import Vector3Stamped


# ----------------- Quaternion / Euler utils -----------------
def quat_normalize(q):
    w, x, y, z = q
    n = math.sqrt(w*w + x*x + y*y + z*z)
    if n == 0.0:
        return (1.0, 0.0, 0.0, 0.0)
    inv = 1.0 / n
    return (w*inv, x*inv, y*inv, z*inv)

def quat_multiply(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return (
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    )

def quat_from_euler(roll, pitch, yaw):
    cr = math.cos(roll*0.5); sr = math.sin(roll*0.5)
    cp = math.cos(pitch*0.5); sp = math.sin(pitch*0.5)
    cy = math.cos(yaw*0.5); sy = math.sin(yaw*0.5)
    w = cr*cp*cy + sr*sp*sy
    x = sr*cp*cy - cr*sp*sy
    y = cr*sp*cy + sr*cp*sy
    z = cr*cp*sy - sr*sp*cy
    return quat_normalize((w, x, y, z))

def euler_from_quat(q):
    w, x, y, z = q
    # roll (x-axis)
    sinr_cosp = 2*(w*x + y*z)
    cosr_cosp = 1 - 2*(x*x + y*y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    # pitch (y-axis)
    sinp = 2*(w*y - z*x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi/2, sinp)
    else:
        pitch = math.asin(sinp)
    # yaw (z-axis)
    siny_cosp = 2*(w*z + x*y)
    cosy_cosp = 1 - 2*(y*y + z*z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


# ----------------- Madgwick filter -----------------
class Madgwick:
    """ Minimal MadgwickAHRS (IMU+Mag) implementation. """
    def __init__(self, beta=0.1):
        self.beta = beta
        self.q = (1.0, 0.0, 0.0, 0.0)

    def update(self, gx, gy, gz, ax, ay, az, mx, my, mz, dt):
        # Normalize accelerometer
        an = math.sqrt(ax*ax + ay*ay + az*az)
        if an == 0.0:
            return self.q
        ax, ay, az = ax/an, ay/an, az/an

        # Normalize magnetometer
        mn = math.sqrt(mx*mx + my*my + mz*mz)
        if mn == 0.0:
            # fallback to IMU-only update if mag invalid
            return self.update_imu(gx, gy, gz, ax, ay, az, dt)
        mx, my, mz = mx/mn, my/mn, mz/mn

        q1, q2, q3, q4 = self.q  # (w, x, y, z)

        # Auxiliary variables
        hx = mx*(q1*q1 + q2*q2 - q3*q3 - q4*q4) + 2.0*my*(q2*q3 - q1*q4) + 2.0*mz*(q2*q4 + q1*q3)
        hy = 2.0*mx*(q2*q3 + q1*q4) + my*(q1*q1 - q2*q2 + q3*q3 - q4*q4) + 2.0*mz*(q3*q4 - q1*q2)
        _2bx = math.sqrt(hx*hx + hy*hy)
        _2bz = 2.0*mx*(q2*q4 - q1*q3) + 2.0*my*(q1*q2 + q3*q4) + mz*(q1*q1 - q2*q2 - q3*q3 + q4*q4)
        _4bx = 2.0*_2bx
        _4bz = 2.0*_2bz

        # Gradient descent step
        s1 = (-2.0*(q3*(2.0*q2*q4 - 2.0*q1*q3 - ax) - q2*(2.0*q1*q2 + 2.0*q3*q4 - ay))
              - _2bz*q3*(_2bx*(0.5 - q3*q3 - q4*q4) + _2bz*(q2*q4 - q1*q3) - mx)
              + (-_2bx*q4 + _2bz*q2)*(_2bx*(q2*q3 - q1*q4) + _2bz*(q1*q2 + q3*q4) - my)
              + _2bx*q3*(_2bx*(q1*q3 + q2*q4) + _2bz*(0.5 - q2*q2 - q3*q3) - mz))
        s2 = ( 2.0*(q4*(2.0*q2*q4 - 2.0*q1*q3 - ax) + q1*(2.0*q1*q2 + 2.0*q3*q4 - ay))
              - _2bz*q4*(_2bx*(0.5 - q3*q3 - q4*q4) + _2bz*(q2*q4 - q1*q3) - mx)
              + (_2bx*q3 + _2bz*q1)*(_2bx*(q2*q3 - q1*q4) + _2bz*(q1*q2 + q3*q4) - my)
              + (_2bx*q4 - _4bz*q2)*(_2bx*(q1*q3 + q2*q4) + _2bz*(0.5 - q2*q2 - q3*q3) - mz))
        s3 = (-2.0*(q1*(2.0*q2*q4 - 2.0*q1*q3 - ax) - q4*(2.0*q1*q2 + 2.0*q3*q4 - ay))
              + (-_4bx*q3 - _2bz*q1)*(_2bx*(0.5 - q3*q3 - q4*q4) + _2bz*(q2*q4 - q1*q3) - mx)
              + (_2bx*q2 + _2bz*q4)*(_2bx*(q2*q3 - q1*q4) + _2bz*(q1*q2 + q3*q4) - my)
              + (_2bx*q1 - _4bz*q3)*(_2bx*(q1*q3 + q2*q4) + _2bz*(0.5 - q2*q2 - q3*q3) - mz))
        s4 = ( 2.0*(q2*(2.0*q2*q4 - 2.0*q1*q3 - ax) + q3*(2.0*q1*q2 + 2.0*q3*q4 - ay))
              + (-_4bx*q4 + _2bz*q2)*(_2bx*(0.5 - q3*q3 - q4*q4) + _2bz*(q2*q4 - q1*q3) - mx)
              + (-_2bx*q1 + _2bz*q3)*(_2bx*(q2*q3 - q1*q4) + _2bz*(q1*q2 + q3*q4) - my)
              + _2bx*q2*(_2bx*(q1*q3 + q2*q4) + _2bz*(0.5 - q2*q2 - q3*q3) - mz))
        norm = math.sqrt(s1*s1 + s2*s2 + s3*s3 + s4*s4)
        if norm == 0.0:
            return self.q
        s1, s2, s3, s4 = s1/norm, s2/norm, s3/norm, s4/norm

        # Rate of change from gyro
        qw_dot = 0.5 * (-q2*gx - q3*gy - q4*gz) - self.beta*s1
        qx_dot = 0.5 * ( q1*gx + q3*gz - q4*gy) - self.beta*s2
        qy_dot = 0.5 * ( q1*gy - q2*gz + q4*gx) - self.beta*s3
        qz_dot = 0.5 * ( q1*gz + q2*gy - q3*gx) - self.beta*s4

        q1 += qw_dot * dt
        q2 += qx_dot * dt
        q3 += qy_dot * dt
        q4 += qz_dot * dt

        self.q = quat_normalize((q1, q2, q3, q4))
        return self.q

    def update_imu(self, gx, gy, gz, ax, ay, az, dt):
        an = math.sqrt(ax*ax + ay*ay + az*az)
        if an == 0.0:
            return self.q
        ax, ay, az = ax/an, ay/an, az/an

        q1, q2, q3, q4 = self.q
        # gradient step (reference gravity [0,0,1])
        f1 = 2*(q2*q4 - q1*q3) - ax
        f2 = 2*(q1*q2 + q3*q4) - ay
        f3 = 2*(0.5 - q2*q2 - q3*q3) - az

        s1 = -2*q3*f1 + 2*q2*f2
        s2 =  2*q4*f1 + 2*q1*f2 - 4*q2*f3
        s3 = -2*q1*f1 + 2*q4*f2 - 4*q3*f3
        s4 =  2*q2*f1 + 2*q3*f2
        norm = math.sqrt(s1*s1 + s2*s2 + s3*s3 + s4*s4)
        if norm != 0.0:
            s1, s2, s3, s4 = s1/norm, s2/norm, s3/norm, s4/norm

        # integrate
        qw_dot = 0.5 * (-q2*gx - q3*gy - q4*gz) - self.beta*s1
        qx_dot = 0.5 * ( q1*gx + q3*gz - q4*gy) - self.beta*s2
        qy_dot = 0.5 * ( q1*gy - q2*gz + q4*gx) - self.beta*s3
        qz_dot = 0.5 * ( q1*gz + q2*gy - q3*gx) - self.beta*s4

        q1 += qw_dot * dt
        q2 += qx_dot * dt
        q3 += qy_dot * dt
        q4 += qz_dot * dt

        self.q = quat_normalize((q1, q2, q3, q4))
        return self.q


# ----------------- ROS2 Node -----------------
class IMUNode(Node):
    def __init__(self):
        super().__init__('imu_node')

        # ---- Parameters ----
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 1000000)
        self.declare_parameter('topic', '/imu/rpy')
        self.declare_parameter('format', 'raw10')      # 'raw10' | 'razor_ypr' | 'csv_rpy'
        self.declare_parameter('unit', 'deg')          # 'deg' | 'rad'
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('debug', False)

        # Fusion params
        self.declare_parameter('beta', 0.3)            # 튜닝: 0.2~0.4 권장(정지 안정성↑)
        self.declare_parameter('gyro_unit', 'dps')     # 'dps' or 'rads'
        self.declare_parameter('use_packet_time', True)
        self.declare_parameter('dt_max', 0.1)          # dt 상한(초)

        # Scale params (센서 설정에 맞게 조정)
        self.declare_parameter('acc_lsb_per_g', 16384.0)            # ±2g
        self.declare_parameter('gyro_lsb_per_dps', 131.0)           # ±250 dps
        self.declare_parameter('mag_uT_per_lsb', 4912.0/32760.0)    # AK8963 16bit

        # Gyro bias estimator
        self.declare_parameter('bias_collect_sec', 2.0)   # 정지 상태 수집 시간(초)
        self.declare_parameter('bias_g_thresh', 0.5)      # dps 기준(스케일 적용 후)
        self.declare_parameter('bias_acc_1g_tol', 0.05)   # | |acc|-1 | 허용오차

        # Load parameters
        self.port     = self.get_parameter('port').value
        self.baud     = int(self.get_parameter('baud').value)
        self.topic    = self.get_parameter('topic').value
        self.format   = self.get_parameter('format').value
        self.unit     = self.get_parameter('unit').value
        self.frame_id = self.get_parameter('frame_id').value
        self.debug    = bool(self.get_parameter('debug').value)

        self.beta     = float(self.get_parameter('beta').value)
        self.gyro_unit = self.get_parameter('gyro_unit').value
        self.use_packet_time = bool(self.get_parameter('use_packet_time').value)
        self.dt_max   = float(self.get_parameter('dt_max').value)

        self.acc_lsb_per_g     = float(self.get_parameter('acc_lsb_per_g').value)
        self.gyro_lsb_per_dps  = float(self.get_parameter('gyro_lsb_per_dps').value)
        self.mag_uT_per_lsb    = float(self.get_parameter('mag_uT_per_lsb').value)

        self.bias_collect_sec  = float(self.get_parameter('bias_collect_sec').value)
        self.bias_g_thresh     = float(self.get_parameter('bias_g_thresh').value)
        self.bias_acc_1g_tol   = float(self.get_parameter('bias_acc_1g_tol').value)

        # ---- Publisher ----
        self.pub = self.create_publisher(
            Vector3Stamped, self.topic, QoSPresetProfiles.SENSOR_DATA.value
        )

        # ---- Serial / thread ----
        self._ser = None
        self._run = True
        self._th = threading.Thread(target=self.reader, daemon=True)
        self._th.start()

        # ---- Fusion state ----
        self.filter = Madgwick(beta=self.beta)
        self.prev_t = None
        self.host_prev_ts = None

        # Gyro bias
        self.gyro_bias = [0.0, 0.0, 0.0]
        self.bias_samples = []
        self.bias_lockin = False
        self.bias_start_time = None  # host time to limit collection window

        self.get_logger().info(
            f'IMU node started: {self.port} @ {self.baud}, format={self.format}, unit={self.unit}'
        )

    # --- utilities ---
    def open_serial(self):
        while self._run and self._ser is None:
            try:
                self._ser = serial.Serial(self.port, self.baud, timeout=0.1)
                self.get_logger().info('Serial opened.')
            except Exception as e:
                self.get_logger().warning(f'Open failed: {e}; retry in 1s')
                time.sleep(1.0)

    def _read_line(self):
        """Read one line from serial handling both LF and CR endings."""
        if self._ser is None or not self._ser.is_open:
            return ''
        line_bytes = self._ser.read_until(b'\n', 256)
        if not line_bytes:
            line_bytes = self._ser.read_until(b'\r', 256)
        if not line_bytes:
            return ''
        return line_bytes.decode(errors='ignore').strip().replace('\r', '')

    @staticmethod
    def _wrap180(deg):
        return (deg + 180.0) % 360.0 - 180.0

    @staticmethod
    def _clamp(v, lo, hi):
        return max(lo, min(hi, v))

    def _publish_rpy(self, roll, pitch, yaw):
        # roll, pitch, yaw: radians in ZYX (yaw-pitch-roll) from euler_from_quat()
        if self.unit == 'deg':
            r2d = 180.0 / math.pi
            roll  *= r2d
            pitch *= r2d
            yaw   *= r2d

            # 기준 범위 적용
            roll  = self._wrap180(roll)                 # [-180, 180)
            yaw   = self._wrap180(yaw)                  # [-180, 180)
            pitch = self._clamp(pitch, -90.0, 90.0)     # [-90, 90]
        else:
            # 라디안 기준
            roll  = (roll + math.pi) % (2*math.pi) - math.pi     # [-π, π)
            yaw   = (yaw  + math.pi) % (2*math.pi) - math.pi     # [-π, π)
            pitch = self._clamp(pitch, -math.pi/2, math.pi/2)         # [-π/2, π/2]

        msg = Vector3Stamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.vector.x = float(roll)
        msg.vector.y = float(pitch)
        msg.vector.z = float(yaw)
        self.pub.publish(msg)

        if self.debug:
            u = 'deg' if self.unit == 'deg' else 'rad'
            self.get_logger().info(f'PUB r={roll:.2f} p={pitch:.2f} y={yaw:.2f} ({u})')



    def _apply_scales(self, ax, ay, az, gx, gy, gz, mx, my, mz):
        """Convert raw LSB counts to physical units (g, dps, uT)."""
        ax, ay, az = ax / self.acc_lsb_per_g, ay / self.acc_lsb_per_g, az / self.acc_lsb_per_g
        gx, gy, gz = gx / self.gyro_lsb_per_dps, gy / self.gyro_lsb_per_dps, gz / self.gyro_lsb_per_dps
        mx, my, mz = mx * self.mag_uT_per_lsb, my * self.mag_uT_per_lsb, mz * self.mag_uT_per_lsb
        return ax, ay, az, gx, gy, gz, mx, my, mz

    def _maybe_collect_bias(self, gx, gy, gz, ax, ay, az):
        """Collect gyro bias during initial 'still' window."""
        if self.bias_lockin:
            return

        now = time.monotonic()
        if self.bias_start_time is None:
            self.bias_start_time = now

        # 정지 판단: |acc|-1g가 작고, gyro가 작을 때
        g_norm = math.sqrt(gx*gx + gy*gy + gz*gz)
        a_norm = math.sqrt(ax*ax + ay*ay + az*az)
        if abs(a_norm - 1.0) < self.bias_acc_1g_tol and g_norm < self.bias_g_thresh:
            self.bias_samples.append((gx, gy, gz))

        if (now - self.bias_start_time) >= self.bias_collect_sec:
            if len(self.bias_samples) > 10:
                bx = sum(s[0] for s in self.bias_samples) / len(self.bias_samples)
                by = sum(s[1] for s in self.bias_samples) / len(self.bias_samples)
                bz = sum(s[2] for s in self.bias_samples) / len(self.bias_samples)
                self.gyro_bias = [bx, by, bz]
                self.bias_lockin = True
                if self.debug:
                    self.get_logger().info(f'Gyro bias locked: [{bx:.4f}, {by:.4f}, {bz:.4f}] dps')
            else:
                # 수집 실패 → 다음 윈도에 다시 시도
                self.bias_start_time = now
                self.bias_samples.clear()

    # --- main reader loop ---
    def reader(self):
        self.open_serial()
        self.host_prev_ts = time.monotonic()
        while self._run:
            try:
                if self._ser is None or not self._ser.is_open:
                    self._ser = None
                    self.open_serial()
                    continue

                line = self._read_line()
                if not line:
                    continue
                if self.debug:
                    self.get_logger().info(f'RAW: {line}')

                q = None  # quaternion

                if self.format == 'razor_ypr':
                    # "YPR=yaw,pitch,roll"  (degrees)
                    if line.startswith('YPR=') and ',' in line:
                        try:
                            ypr = line.split('=', 1)[1]
                            y, p, r = [float(x) for x in ypr.split(',')]
                            d2r = math.pi / 180.0
                            yaw, pitch, roll = y * d2r, p * d2r, r * d2r
                            q = quat_from_euler(roll, pitch, yaw)
                        except Exception as e:
                            if self.debug:
                                self.get_logger().warning(f'Parse fail(razor_ypr): {line} ({e})')
                            continue

                elif self.format == 'csv_rpy':
                    # "roll,pitch,yaw"  (degrees)
                    try:
                        parts = [float(x) for x in line.split(',')]
                        if len(parts) == 3:
                            r, p, y = parts
                            d2r = math.pi / 180.0
                            roll, pitch, yaw = r * d2r, p * d2r, y * d2r
                            q = quat_from_euler(roll, pitch, yaw)
                        else:
                            continue
                    except Exception as e:
                        if self.debug:
                            self.get_logger().warning(f'Parse fail(csv_rpy): {line} ({e})')
                        continue

                elif self.format == 'raw10':
                    # "t, ax, ay, az, gx, gy, gz, mx, my, mz" (raw LSBs)
                    try:
                        vals = [float(x) for x in line.split(',')]
                        if len(vals) != 10:
                            continue
                        t_raw, ax, ay, az, gx, gy, gz, mx, my, mz = vals

                        # 1) scale to physical units
                        ax, ay, az, gx, gy, gz, mx, my, mz = self._apply_scales(ax, ay, az, gx, gy, gz, mx, my, mz)

                        # 2) dt (packet or host), with clamp
                        if self.use_packet_time:
                            if self.prev_t is None:
                                dt = 0.01
                            else:
                                # if looks like ms counter (e.g., >1e6), convert to seconds
                                dt_raw = (t_raw - self.prev_t) * (1e-3 if t_raw > 1e6 else 1.0)
                                dt = max(1e-4, min(dt_raw, self.dt_max))
                        else:
                            now = time.monotonic()
                            dt = max(1e-4, min(now - (self.host_prev_ts or now), self.dt_max))
                            self.host_prev_ts = now
                        self.prev_t = t_raw

                        # 3) bias collection (in dps units)
                        self._maybe_collect_bias(gx, gy, gz, ax, ay, az)

                        # 4) remove gyro bias
                        gx -= self.gyro_bias[0]
                        gy -= self.gyro_bias[1]
                        gz -= self.gyro_bias[2]

                        # 5) convert gyro to rad/s if needed
                        if self.gyro_unit.lower().startswith('dps'):
                            d2r = math.pi/180.0
                            gx, gy, gz = gx*d2r, gy*d2r, gz*d2r

                        # 6) fuse
                        q = self.filter.update(gx, gy, gz, ax, ay, az, mx, my, mz, dt)

                    except Exception as e:
                        if self.debug:
                            self.get_logger().warning(f'Parse fail(raw10): {line} ({e})')
                        continue

                else:
                    if self.debug:
                        self.get_logger().warning(f'Unknown format={self.format} line={line}')
                    continue

                if q is None:
                    continue

                # quaternion -> RPY
                roll, pitch, yaw = euler_from_quat(q)
                self._publish_rpy(roll, pitch, yaw)

            except serial.SerialException as e:
                self.get_logger().error(f'Serial error: {e}')
                try:
                    if self._ser:
                        self._ser.close()
                except Exception:
                    pass
                self._ser = None
                time.sleep(1.0)
            except Exception as e:
                if getattr(self, 'debug', False):
                    self.get_logger().warning(f'Loop warn: {e}')
                time.sleep(0.01)

    # --- cleanup ---
    def destroy_node(self):
        self._run = False
        try:
            if self._th.is_alive():
                self._th.join(timeout=0.2)
        except Exception:
            pass
        try:
            if self._ser and self._ser.is_open:
                self._ser.close()
        except Exception:
            pass
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = IMUNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
