# gamepad_ble.py - OPTIMIZED for MicroPython (ESP32-C3)
# Frame: G,bX,bO,bS,bT,dU,dD,dL,dR,L1,R1,jLx,jLy,jRx,jRy\n
# Buttons/Dpad/Shoulder: 0/1, Joystick: -100..100

import os, time
from bluetooth import BLE

try:
    from ble_uart_peripheral import BLEUART
except ImportError:
    raise ImportError("Missing ble_uart_peripheral.py (BLEUART). Upload it to ESP32.")


def _clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


_gp = None


class _GamepadBLE:
    def __init__(self, disable_ota_ble=True, deadzone=6, debug=False,
                 max_buf=2048, compact_threshold=512):
        self.disable_ota_ble = bool(disable_ota_ble)
        self.deadzone = int(deadzone) if deadzone is not None else 0
        self.debug = bool(debug)

        self.max_buf = int(max_buf)
        self.compact_threshold = int(compact_threshold)

        self.ble = BLE()
        self.uart = None

        # ring-buffer style
        self._rx_buf = bytearray()
        self._rx_head = 0
        self._overflow = False

        self._connected = False
        self._ever_rx = False

        # prealloc parsed values (14 fields after 'G,')
        self._vals = [0] * 14

        # state arrays (fast)
        self._btn = [0, 0, 0, 0]   # X O S T
        self._dpad = [0, 0, 0, 0]  # U D L R
        self._sh = [0, 0]          # L1 R1
        self.jlx = 0
        self.jly = 0
        self.jrx = 0
        self.jry = 0

    def log(self, *a):
        if self.debug:
            print(*a)

    def _dz(self, v):
        v = int(_clamp(v, -100, 100))
        if self.deadzone and abs(v) < self.deadzone:
            return 0
        return v

    def start(self, name):
        if self.disable_ota_ble:
            # best effort stop dupterm / other BLE usage
            try:
                os.dupterm(None)
            except:
                pass
            try:
                self.ble.active(False)
            except:
                pass
            time.sleep_ms(150)

        self.ble.active(True)
        self.uart = BLEUART(self.ble, name=name)
        self.uart.irq(handler=self._on_rx)

        self.log("=== Gamepad BLE (optimized) ===")
        self.log("Advertising as:", name)
        try:
            self.uart.write("READY\n")
        except:
            pass

    def is_connected(self):
        if self.uart is None:
            return False
        for attr in ("is_connected", "connected"):
            if hasattr(self.uart, attr):
                try:
                    v = getattr(self.uart, attr)
                    return v() if callable(v) else bool(v)
                except:
                    pass
        return self._connected or self._ever_rx

    def wait_connected(self, timeout_ms=0, poll_ms=50):
        t0 = time.ticks_ms()
        while True:
            if self.is_connected():
                self._connected = True
                return True
            if timeout_ms and time.ticks_diff(time.ticks_ms(), t0) > timeout_ms:
                return False
            time.sleep_ms(poll_ms)

    # IMPORTANT: called inside BLEUART _irq -> must never throw
    def _on_rx(self):
        try:
            data = self.uart.read()
            if not data:
                return
            self._ever_rx = True
            self._rx_buf.extend(data)

            # mark overflow; do trimming in main thread (update)
            if len(self._rx_buf) > self.max_buf:
                self._overflow = True
        except:
            # never crash in IRQ
            try:
                self._rx_buf = bytearray()
                self._rx_head = 0
                self._overflow = False
            except:
                pass

    def _compact_if_needed(self):
        # Handle overflow or large head -> compact in main loop
        if self._overflow:
            keep = self.max_buf // 2
            L = len(self._rx_buf)
            if L > keep:
                # keep last 'keep' bytes
                self._rx_buf = self._rx_buf[L - keep:]
            self._rx_head = 0
            self._overflow = False

        if self._rx_head > self.compact_threshold:
            # drop consumed prefix
            self._rx_buf = self._rx_buf[self._rx_head:]
            self._rx_head = 0

    def update(self, max_lines=3):
        """
        Parse up to max_lines full lines from RX buffer.
        Return number of parsed snapshot frames.
        """
        self._compact_if_needed()

        n = 0
        buf = self._rx_buf
        head = self._rx_head

        while n < max_lines:
            nl = buf.find(b"\n", head)
            if nl < 0:
                break

            raw = buf[head:nl]  # bytearray slice
            head = nl + 1

            if not raw:
                continue
            # strip \r
            if raw[-1] == 13:
                raw = raw[:-1]

            # Ping
            if raw == b"P":
                try:
                    self.uart.write("PONG\n")
                except:
                    pass
                continue

            # Snapshot starts with b'G,'
            if len(raw) >= 2 and raw[0] == 71 and raw[1] == 44:
                if self._apply_snapshot_bytes(raw):
                    n += 1

        self._rx_head = head
        # compact after parsing too
        self._compact_if_needed()
        return n

    def _parse14(self, raw):
        # parse 14 ints after 'G,' into self._vals
        vals = self._vals
        idx = 0
        num = 0
        sign = 1
        in_num = False

        i = 2
        L = len(raw)
        while i < L and idx < 14:
            c = raw[i]
            if c == 45:  # '-'
                sign = -1
            elif 48 <= c <= 57:  # '0'..'9'
                num = num * 10 + (c - 48)
                in_num = True
            elif c == 44:  # ','
                if in_num:
                    vals[idx] = sign * num
                    idx += 1
                sign = 1
                num = 0
                in_num = False
            i += 1

        if idx < 14 and in_num:
            vals[idx] = sign * num
            idx += 1

        return idx == 14

    def _apply_snapshot_bytes(self, raw):
        if not self._parse14(raw):
            return False

        v = self._vals

        # buttons: X O S T
        self._btn[0] = 1 if v[0] else 0
        self._btn[1] = 1 if v[1] else 0
        self._btn[2] = 1 if v[2] else 0
        self._btn[3] = 1 if v[3] else 0

        # dpad: U D L R
        self._dpad[0] = 1 if v[4] else 0
        self._dpad[1] = 1 if v[5] else 0
        self._dpad[2] = 1 if v[6] else 0
        self._dpad[3] = 1 if v[7] else 0

        # shoulders: L1 R1
        self._sh[0] = 1 if v[8] else 0
        self._sh[1] = 1 if v[9] else 0

        # joysticks
        self.jlx = self._dz(v[10])
        self.jly = self._dz(v[11])
        self.jrx = self._dz(v[12])
        self.jry = self._dz(v[13])
        return True

    # ---- reads ----
    def read_button(self, k):
        if isinstance(k, int):
            return self._btn[k] if 0 <= k <= 3 else 0
        s = (k or "").upper()
        if s == "X": return self._btn[0]
        if s == "O": return self._btn[1]
        if s == "S": return self._btn[2]
        if s == "T": return self._btn[3]
        return 0

    def read_dpad(self, k):
        if isinstance(k, int):
            return self._dpad[k] if 0 <= k <= 3 else 0
        s = (k or "").upper()
        if s == "U": return self._dpad[0]
        if s == "D": return self._dpad[1]
        if s == "L": return self._dpad[2]
        if s == "R": return self._dpad[3]
        return 0

    def read_shoulder(self, k):
        if isinstance(k, int):
            return self._sh[k] if 0 <= k <= 1 else 0
        s = (k or "").upper()
        if s == "L1": return self._sh[0]
        if s == "R1": return self._sh[1]
        return 0

    def read_joystick_xy(self, side):
        s = (side or "").upper()
        return (self.jlx, self.jly) if s == "L" else (self.jrx, self.jry)

    def read_joy_x(self, side): return self.read_joystick_xy(side)[0]
    def read_joy_y(self, side): return self.read_joystick_xy(side)[1]

    def read_joy_distance(self, side):
        x, y = self.read_joystick_xy(side)
        d = int((x * x + y * y) ** 0.5 + 0.5)
        return 100 if d > 100 else d

    def read_joy_angle(self, side):
        try:
            import math
        except:
            return 0
        x, y = self.read_joystick_xy(side)
        if x == 0 and y == 0:
            return 0
        a = math.degrees(math.atan2(y, x))
        if a < 0:
            a += 360
        return int(a) % 360


# module API
def start(name="MEBLOCK-GAMEPAD", disable_ota_ble=True, deadzone=6, debug=False):
    global _gp
    _gp = _GamepadBLE(disable_ota_ble=disable_ota_ble, deadzone=deadzone, debug=debug)
    _gp.start(name)
    return _gp

def wait_connected(timeout_ms=0):
    if _gp is None:
        raise RuntimeError("Call gamepad_ble.start() first.")
    return _gp.wait_connected(timeout_ms=timeout_ms)

def update(max_lines=3):
    return 0 if _gp is None else _gp.update(max_lines=max_lines)

def read_button(k): return 0 if _gp is None else _gp.read_button(k)
def read_dpad(k): return 0 if _gp is None else _gp.read_dpad(k)
def read_shoulder(k): return 0 if _gp is None else _gp.read_shoulder(k)
def read_joystick_xy(side): return (0, 0) if _gp is None else _gp.read_joystick_xy(side)
def read_joy_x(side): return 0 if _gp is None else _gp.read_joy_x(side)
def read_joy_y(side): return 0 if _gp is None else _gp.read_joy_y(side)
def read_joy_distance(side): return 0 if _gp is None else _gp.read_joy_distance(side)
def read_joy_angle(side): return 0 if _gp is None else _gp.read_joy_angle(side)
