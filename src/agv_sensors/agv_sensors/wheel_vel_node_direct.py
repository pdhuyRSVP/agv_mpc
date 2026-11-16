#!/usr/bin/env python3
"""
wheel_vel_node_direct.py - agv_sensors (USB-RS485 direct)
=========================================================================
Architecture:
  /cmd_vel -> deque(maxlen=1) -> Modbus thread -> ELD2-RS drivers
  ELD2-RS actual RPM -> TwistWithCovarianceStamped /wheel_vel

This node talks directly to two ELD2-RS drivers over a USB-RS485 adapter
using Modbus RTU. It replaces the old microcontroller middle layer.

Lý do 1 thread duy nhất cho Modbus:
  RS485 là half-duplex, 1 master trên bus. Nếu dùng 2 thread (read/write)
  sẽ bị collision trên bus → CRC error → timeout → giật robot.
  1 thread sequential: write_L → write_R → read_L → read_R → lặp lại.
  Không cần mutex, không collision.

Timing ở 115200 baud (khuyến nghị cho Modbus):
  1 frame FC16 (1 reg) = 8+8 = ~2ms round-trip
  1 frame FC03 (1 reg) = 8+7 = ~2ms round-trip
  4 transactions × ~4ms = ~16ms/cycle → 50 Hz thực tế.
  (57600 baud: ~8ms/transaction → ~32ms/cycle → 30 Hz)

ELD2-RS Modbus registers:
  SID_LEFT  = 1
  SID_RIGHT = 2
  PATH0 base    = 0x6200  (mode=velocity 0x0002)
  PATH0 speed   = 0x6203  (int16, RPM motor)
  PATH0 trigger = 0x6207  (0x0010 để execute)
  Actual vel    = 0x0B06  (int16, RPM motor)
"""

import collections
import math
import struct
import threading
import time
from typing import Optional

import rclpy
import serial
from geometry_msgs.msg import Twist, TwistWithCovarianceStamped
from rclpy.node import Node
from std_msgs.msg import String

# ─── Covariance thresholds ────────────────────────────────────────────────────
_MOVING_VX       = 0.005   # m/s
_MOVING_WZ       = 0.01    # rad/s
_COV_VX_STILL    = 0.001
_COV_WZ_STILL    = 0.001
_COV_VX_MOVE     = 0.01
_COV_WZ_MOVE     = 0.01

# ─── Modbus constants ─────────────────────────────────────────────────────────
_SID_LEFT        = 1
_SID_RIGHT       = 2
_REG_PATH0_BASE  = 0x6200
_REG_PATH0_SPEED = 0x6203
_REG_PATH0_TRIG  = 0x6207
_REG_ACT_VEL     = 0x0B06
_TRIG_VAL        = 0x0010

# ─── Timing ───────────────────────────────────────────────────────────────────
_MODBUS_TIMEOUT_S   = 0.030   # 30ms per transaction (generous for 115200)
_INTER_FRAME_S      = 0.002   # t3.5 gap ≥ 3.5 char-times (at 115200 ≈ 0.3ms → use 2ms)
_LOOP_HZ            = 20.0    # target bus loop rate (write+read cycle)
_WATCHDOG_S         = 3.0
_RECONNECT_DELAY_S  = 2.0
_CMD_TIMEOUT_S      = 0.5     # nếu không có cmd_vel mới sau 0.5s → gửi 0,0


# =============================================================================
# CRC16 Modbus
# =============================================================================

def _crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc


def _pack_crc(payload: bytes) -> bytes:
    c = _crc16(payload)
    return payload + bytes([c & 0xFF, c >> 8])


def _check_crc(frame: bytes) -> bool:
    if len(frame) < 3:
        return False
    recv_crc = frame[-2] | (frame[-1] << 8)
    return recv_crc == _crc16(frame[:-2])


def _as_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ('1', 'true', 'yes', 'on')
    return bool(value)


def _as_float(value) -> float:
    return float(value)


def _as_int(value) -> int:
    return int(float(value))


# =============================================================================
# Modbus frame builders
# =============================================================================

def _build_fc16(sid: int, reg: int, values: list[int]) -> bytes:
    """FC16 Write Multiple Registers. values = list of uint16."""
    qty = len(values)
    hdr = bytes([sid, 0x10,
                 (reg >> 8) & 0xFF, reg & 0xFF,
                 (qty >> 8) & 0xFF, qty & 0xFF,
                 qty * 2])
    data = b''
    for v in values:
        u = v & 0xFFFF
        data += bytes([(u >> 8) & 0xFF, u & 0xFF])
    return _pack_crc(hdr + data)


def _build_fc03(sid: int, reg: int, qty: int) -> bytes:
    """FC03 Read Holding Registers."""
    payload = bytes([sid, 0x03,
                     (reg >> 8) & 0xFF, reg & 0xFF,
                     (qty >> 8) & 0xFF, qty & 0xFF])
    return _pack_crc(payload)


# =============================================================================
# Low-level Modbus transactions (blocking, called from Modbus thread only)
# =============================================================================

class ModbusRTU:
    """
    Thin synchronous Modbus RTU wrapper cho 1 serial port.
    Tất cả method phải gọi từ cùng 1 thread (không thread-safe theo thiết kế
    để tránh overhead – Modbus thread là owner duy nhất).
    """

    def __init__(self, port: str, baud: int, timeout_s: float,
                 inter_frame_s: float, logger):
        self._port          = port
        self._baud          = baud
        self._timeout_s     = timeout_s
        self._inter_s       = inter_frame_s
        self._log           = logger
        self.ser: Optional[serial.Serial] = None

    def open(self) -> bool:
        try:
            self.ser = serial.Serial(
                port=self._port,
                baudrate=self._baud,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=self._timeout_s,
            )
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            self._log.info(f"[Modbus] Serial mở: {self._port} @ {self._baud}")
            return True
        except Exception as e:
            self._log.error(f"[Modbus] Không mở được {self._port}: {e}")
            self.ser = None
            return False

    def close(self):
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass
        self.ser = None

    def is_open(self) -> bool:
        return self.ser is not None and self.ser.is_open

    def _send_recv(self, request: bytes, expect_bytes: int) -> Optional[bytes]:
        """
        Gửi request, đọc expect_bytes bytes response.
        Return None nếu timeout / CRC lỗi / exception.
        """
        if not self.is_open():
            return None
        try:
            # Xả buffer trước khi gửi (tránh stale response từ lần trước)
            self.ser.reset_input_buffer()
            # Inter-frame gap
            time.sleep(self._inter_s)
            # Send
            self.ser.write(request)
            # Receive
            resp = self.ser.read(expect_bytes)
            if len(resp) < expect_bytes:
                return None   # timeout
            if not _check_crc(resp):
                self._log.debug(f"[Modbus] CRC lỗi: {resp.hex()}")
                return None
            return resp
        except serial.SerialException as e:
            self._log.error(f"[Modbus] SerialException: {e}")
            self.close()
            return None
        except Exception as e:
            self._log.warn(f"[Modbus] _send_recv lỗi: {e}")
            return None

    def write_single_reg(self, sid: int, reg: int, value: int) -> bool:
        """
        FC16 ghi 1 register.  value = int16 (signed OK do mask & 0xFFFF).
        """
        req  = _build_fc16(sid, reg, [value & 0xFFFF])
        resp = self._send_recv(req, 8)
        if resp is None:
            return False
        # Validate echo: sid, FC=0x10, reg hi/lo, qty hi/lo
        if resp[0] != sid or resp[1] != 0x10:
            return False
        return True

    def write_speed_and_trigger(self, sid: int, rpm: int) -> bool:
        """
        Ghi speed (0x6203) + trigger (0x6207) = 2 transactions.
        Return True nếu cả 2 thành công.
        """
        ok1 = self.write_single_reg(sid, _REG_PATH0_SPEED, rpm)
        ok2 = self.write_single_reg(sid, _REG_PATH0_TRIG,  _TRIG_VAL)
        return ok1 and ok2

    def read_actual_rpm(self, sid: int) -> Optional[int]:
        """
        FC03 đọc 0x0B06 (1 reg).
        Return int16 RPM, hoặc None nếu lỗi.
        """
        req      = _build_fc03(sid, _REG_ACT_VEL, 1)
        exp_len  = 5 + 2 * 1   # = 7
        resp     = self._send_recv(req, exp_len)
        if resp is None:
            return None
        if resp[0] != sid or resp[1] != 0x03 or resp[2] != 2:
            return None
        raw = struct.unpack('>H', resp[3:5])[0]
        return struct.unpack('>h', struct.pack('>H', raw))[0]  # uint16 → int16

    def init_path0(self, sid: int) -> bool:
        """
        FC16 khởi tạo PATH0 velocity mode (8 registers từ 0x6200).
        vals: mode=velocity(2), 0, 0, speed=0, accel=400, decel=400, pause=0, trig=0
        """
        vals = [0x0002, 0x0000, 0x0000, 0x0000,
                0x0190, 0x0190, 0x0000, 0x0000]
        req  = _build_fc16(sid, _REG_PATH0_BASE, vals)
        resp = self._send_recv(req, 8)
        if resp is None:
            return False
        return resp[0] == sid and resp[1] == 0x10


# =============================================================================
# ROS2 Node
# =============================================================================

class WheelVelNode(Node):

    def __init__(self):
        super().__init__('wheel_vel_node')

        # ── Parameters ───────────────────────────────────────────────────────
        self.declare_parameter(
            'port',
            '/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A5069RR4-if00-port0')
        self.declare_parameter('baud',             115200)
        self.declare_parameter('modbus_timeout_ms', int(_MODBUS_TIMEOUT_S * 1000))
        self.declare_parameter('inter_frame_ms',    int(_INTER_FRAME_S  * 1000))
        self.declare_parameter('loop_hz',           _LOOP_HZ)
        self.declare_parameter('wheel_radius',      0.1)
        self.declare_parameter('wheel_separation',  0.636)
        self.declare_parameter('gear_ratio',        10.0)
        self.declare_parameter('base_frame',        'Base_footprint')
        self.declare_parameter('invert_right_fb',   True)   # đảo chiều feedback bánh phải
        self.declare_parameter('invert_right_cmd',  False)  # đảo chiều command bánh phải
        self.declare_parameter('cmd_rpm_clamp',     3000)
        self.declare_parameter('init_path0',        True)

        port            = self.get_parameter('port').value
        baud            = _as_int(self.get_parameter('baud').value)
        mb_timeout_ms   = _as_float(self.get_parameter('modbus_timeout_ms').value)
        inter_ms        = _as_float(self.get_parameter('inter_frame_ms').value)
        loop_hz         = _as_float(self.get_parameter('loop_hz').value)

        self.wheel_radius      = _as_float(self.get_parameter('wheel_radius').value)
        self.wheel_separation  = _as_float(self.get_parameter('wheel_separation').value)
        self.gear_ratio        = _as_float(self.get_parameter('gear_ratio').value)
        self.base_frame        = self.get_parameter('base_frame').value
        self._invert_right_fb  = _as_bool(self.get_parameter('invert_right_fb').value)
        self._invert_right_cmd = _as_bool(self.get_parameter('invert_right_cmd').value)
        self._rpm_clamp        = _as_int(self.get_parameter('cmd_rpm_clamp').value)
        self._do_init_path0    = _as_bool(self.get_parameter('init_path0').value)

        self.get_logger().info(
            f"Init: port={port} baud={baud} "
            f"r={self.wheel_radius}m sep={self.wheel_separation}m "
            f"gear={self.gear_ratio} loop_hz={loop_hz} "
            f"invert_right_fb={self._invert_right_fb}"
        )

        # ── State ─────────────────────────────────────────────────────────────
        self._data_lock      = threading.Lock()
        self._left_rpm_fb    = 0.0
        self._right_rpm_fb   = 0.0
        self._has_fb         = False
        self._last_fb_time   = None
        self._watchdog_warned = False

        self._cmd_deque      = collections.deque(maxlen=1)
        self._last_cmd_time  = 0.0   # monotonic

        self._running        = True

        # ── Modbus ────────────────────────────────────────────────────────────
        self._mb = ModbusRTU(
            port        = port,
            baud        = baud,
            timeout_s   = mb_timeout_ms / 1000.0,
            inter_frame_s = inter_ms / 1000.0,
            logger      = self.get_logger(),
        )

        # ── ROS I/O ───────────────────────────────────────────────────────────
        self.pub_vel = self.create_publisher(
            TwistWithCovarianceStamped, '/wheel_vel', 10)
        self.pub_raw = self.create_publisher(String, '/wheel_vel_raw', 10)
        self.create_subscription(Twist, '/cmd_vel', self._cmd_vel_cb, 10)

        # ── Modbus thread (BUS OWNER) ─────────────────────────────────────────
        self._loop_interval = 1.0 / loop_hz if loop_hz > 0 else 0.05
        self._mb_thread = threading.Thread(
            target=self._modbus_loop, daemon=True, name='wvn_modbus')
        self._mb_thread.start()

        # ── ROS timers ────────────────────────────────────────────────────────
        self.create_timer(0.02, self._publish_cb)    # 50 Hz publish /wheel_vel
        self.create_timer(1.0,  self._watchdog_cb)   # 1  Hz watchdog log

        self.get_logger().info("WheelVelNode READY (direct RS485 mode)")

    # =========================================================================
    # cmd_vel callback  (ROS executor thread – chỉ write deque, không I/O)
    # =========================================================================

    def _cmd_vel_cb(self, msg: Twist):
        v = msg.linear.x
        w = msg.angular.z
        vl = v - w * self.wheel_separation / 2.0
        vr = v + w * self.wheel_separation / 2.0
        rpm_l = self._ms_to_rpm(vl)
        rpm_r = self._ms_to_rpm(vr)
        self._cmd_deque.append((rpm_l, rpm_r))
        self._last_cmd_time = time.monotonic()

    # =========================================================================
    # Modbus thread – SEQUENTIAL bus access, không collision
    # =========================================================================

    def _modbus_loop(self):
        """
        1 thread duy nhất làm chủ RS485 bus:
          - Mở / reconnect serial
          - Init PATH0 nếu cần
          - Mỗi iteration:
              1. Lấy cmd từ deque (hoặc dùng 0,0 nếu cmd timeout)
              2. FC16: ghi speed LEFT  + trigger LEFT
              3. FC16: ghi speed RIGHT + trigger RIGHT
              4. FC03: đọc actual vel LEFT
              5. FC03: đọc actual vel RIGHT
              6. Cập nhật shared data (lock µs)
        """
        path0_inited = False
        consec_err   = 0
        MAX_CONSEC   = 20   # sau 20 lỗi liên tiếp → reconnect

        while self._running and rclpy.ok():
            t0 = time.monotonic()

            # ── 0. Đảm bảo serial mở ─────────────────────────────────────────
            if not self._mb.is_open():
                ok = self._mb.open()
                if not ok:
                    time.sleep(_RECONNECT_DELAY_S)
                    continue
                path0_inited = False
                consec_err   = 0

            # ── 1. Init PATH0 (1 lần sau khi mở port) ────────────────────────
            if self._do_init_path0 and not path0_inited:
                ok_l = self._mb.init_path0(_SID_LEFT)
                ok_r = self._mb.init_path0(_SID_RIGHT)
                if ok_l and ok_r:
                    path0_inited = True
                    self.get_logger().info("[Modbus] PATH0 init OK cả 2 driver")
                else:
                    self.get_logger().warn(
                        f"[Modbus] PATH0 init: L={ok_l} R={ok_r} – thử lại...")
                    time.sleep(0.5)
                    continue

            # ── 2. Lấy lệnh RPM ───────────────────────────────────────────────
            now = time.monotonic()
            if (self._cmd_deque and
                    (now - self._last_cmd_time) < _CMD_TIMEOUT_S):
                rpm_l_raw, rpm_r_raw = self._cmd_deque[-1]
            else:
                rpm_l_raw, rpm_r_raw = 0.0, 0.0   # cmd timeout → dừng bánh

            # Clamp
            rpm_l = int(max(-self._rpm_clamp, min(self._rpm_clamp, rpm_l_raw)))
            rpm_r = int(max(-self._rpm_clamp, min(self._rpm_clamp, rpm_r_raw)))

            if self._invert_right_cmd:
                rpm_r = -rpm_r

            # ── 3. FC16 ghi speed + trigger ───────────────────────────────────
            ok_wl = self._mb.write_speed_and_trigger(_SID_LEFT,  rpm_l)
            ok_wr = self._mb.write_speed_and_trigger(_SID_RIGHT, rpm_r)

            # ── 4. FC03 đọc actual velocity ───────────────────────────────────
            fb_l  = self._mb.read_actual_rpm(_SID_LEFT)
            fb_r  = self._mb.read_actual_rpm(_SID_RIGHT)

            # ── 5. Đếm lỗi / reconnect ────────────────────────────────────────
            if not (ok_wl and ok_wr) or fb_l is None or fb_r is None:
                consec_err += 1
                if consec_err >= MAX_CONSEC:
                    self.get_logger().error(
                        f"[Modbus] {consec_err} lỗi liên tiếp → reconnect serial")
                    self._mb.close()
                    path0_inited = False
                    consec_err   = 0
                    time.sleep(_RECONNECT_DELAY_S)
                    continue
                # Fallback: dùng 0 nếu đọc lỗi, nhưng không reconnect vội
                if fb_l is None: fb_l = 0
                if fb_r is None: fb_r = 0
            else:
                consec_err = 0

            # ── 6. Invert feedback bánh phải (cơ khí đối xứng) ───────────────
            if self._invert_right_fb:
                fb_r = -fb_r

            # ── 7. Publish raw debug string ───────────────────────────────────
            raw_str = f"{fb_l},{fb_r}"
            self.pub_raw.publish(String(data=raw_str))

            # ── 8. Cập nhật shared state (lock chỉ vài µs) ───────────────────
            with self._data_lock:
                self._left_rpm_fb  = float(fb_l)
                self._right_rpm_fb = float(fb_r)
                self._has_fb       = True
                self._last_fb_time = time.monotonic()
                self._watchdog_warned = False

            # ── 9. Sleep còn lại của chu kỳ ──────────────────────────────────
            elapsed = time.monotonic() - t0
            remain  = self._loop_interval - elapsed
            if remain > 0:
                time.sleep(remain)

    # =========================================================================
    # Publish /wheel_vel  (50 Hz ROS timer)
    # =========================================================================

    def _publish_cb(self):
        with self._data_lock:
            if not self._has_fb:
                return
            l_rpm = self._left_rpm_fb
            r_rpm = self._right_rpm_fb

        vl = self._rpm_to_ms(l_rpm)
        vr = self._rpm_to_ms(r_rpm)
        v  = 0.5 * (vr + vl)
        w  = (vr - vl) / self.wheel_separation if self.wheel_separation else 0.0

        moving  = abs(v) > _MOVING_VX or abs(w) > _MOVING_WZ
        cov_vx  = _COV_VX_MOVE  if moving else _COV_VX_STILL
        cov_wz  = _COV_WZ_MOVE  if moving else _COV_WZ_STILL

        msg                          = TwistWithCovarianceStamped()
        msg.header.stamp             = self.get_clock().now().to_msg()
        msg.header.frame_id          = self.base_frame
        msg.twist.twist.linear.x     = v
        msg.twist.twist.angular.z    = w

        cov     = [0.0] * 36
        cov[0]  = cov_vx   # vx
        cov[7]  = 1e6      # vy  (không đo)
        cov[14] = 1e6      # vz  (không đo)
        cov[21] = 1e6      # wx  (không đo)
        cov[28] = 1e6      # wy  (không đo)
        cov[35] = cov_wz   # wz
        msg.twist.covariance = cov

        self.pub_vel.publish(msg)

    # =========================================================================
    # Watchdog  (1 Hz)
    # =========================================================================

    def _watchdog_cb(self):
        with self._data_lock:
            has_fb         = self._has_fb
            last_time      = self._last_fb_time
            already_warned = self._watchdog_warned

        if not has_fb or last_time is None:
            return

        elapsed = time.monotonic() - last_time
        if elapsed > _WATCHDOG_S and not already_warned:
            self.get_logger().warn(
                f"[WATCHDOG] Không nhận được feedback {elapsed:.1f}s! "
                f"Kiểm tra cáp USB-RS485 và driver ELD2-RS (SID 1 & 2).")
            with self._data_lock:
                self._watchdog_warned = True

    # =========================================================================
    # Kinematics
    # =========================================================================

    def _rpm_to_ms(self, rpm: float) -> float:
        wheel_rpm = rpm / self.gear_ratio if self.gear_ratio else rpm
        return wheel_rpm * (2.0 * math.pi / 60.0) * self.wheel_radius

    def _ms_to_rpm(self, v_ms: float) -> float:
        if not self.wheel_radius:
            return 0.0
        return (v_ms * 60.0 / (2.0 * math.pi * self.wheel_radius)) * self.gear_ratio

    # =========================================================================
    # Cleanup
    # =========================================================================

    def destroy_node(self):
        self._running = False
        time.sleep(0.3)
        self._mb.close()
        super().destroy_node()


# =============================================================================
# Entry point
# =============================================================================

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = WheelVelNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
