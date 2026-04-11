# onboard.py - MEBLOCK Onboard Devices (v0.3)
# ------------------------------------------------------------
# Mục tiêu:
# - Nhận diện board (ESP32-S3 / ESP32-C3 / ESP32...)
# - LED onboard là RGB (NeoPixel/WS2812): bật/tắt + đổi màu + nháy
# - Thêm API tạo màu từ RGB và hiển thị/nháy theo màu đã tạo (#RRGGBB)
# - Chừa chỗ để bổ sung các module tích hợp khác theo board
#
# Dùng nhanh:
#   from onboard import rgb, make_color, show, blink_color
#   c = make_color(255, 136, 0)   # "#ff8800"
#   show(c)                       # sáng theo màu đã tạo
#   blink_color(c, 300, times=5)  # nháy 5 lần, chu kỳ 300ms
#
#   rgb.color("RED")   # preset
#   rgb.off()

import os
import sys
import time

try:
    from machine import Pin
except Exception:
    Pin = None

try:
    import neopixel as _neopixel
except Exception:
    _neopixel = None


# =========================
# Board profiles (mở rộng)
# =========================
BOARD_PROFILES = {
    "ESP32S3": {"rgb": {"pin": 48, "n": 1, "order": "RGB"}, "modules": {}},
    "ESP32C3": {"rgb": {"pin": 8,  "n": 1, "order": "GRB"}, "modules": {}},
    "ESP32":   {"rgb": {"pin": 2,  "n": 1, "order": "GRB"}, "modules": {}},
}

# Preset colors
COLORS = {
    "OFF":   (0, 0, 0),
    "BLACK": (0, 0, 0),
    "WHITE": (255, 255, 255),
    "RED":   (255, 0, 0),
    "GREEN": (0, 255, 0),
    "BLUE":  (0, 0, 255),
    "YELLOW": (255, 255, 0),
    "CYAN":   (0, 255, 255),
    "MAGENTA": (255, 0, 255),
    "PURPLE":  (128, 0, 255),
    "PINK":    (255, 20, 147),
    "ORANGE":  (255, 80, 0),
}

def _sleep_ms(ms):
    try:
        time.sleep_ms(int(ms))
    except Exception:
        time.sleep(float(ms) / 1000.0)

def _machine_str():
    try:
        return os.uname().machine
    except Exception:
        pass
    try:
        return getattr(sys.implementation, "_machine", "") or ""
    except Exception:
        return ""

def detect_board_id():
    m = (_machine_str() or "").lower()
    if "esp32s3" in m or "esp32-s3" in m:
        return "ESP32S3"
    if "esp32c3" in m or "esp32-c3" in m:
        return "ESP32C3"
    if "esp32" in m:
        return "ESP32"
    return "UNKNOWN"

def get_profile(board_id=None):
    if board_id is None:
        board_id = detect_board_id()
    return BOARD_PROFILES.get(board_id, {})

def register_board(board_id, rgb_pin=None, rgb_n=None, rgb_order=None, modules=None, **extras):
    if not board_id:
        return

    p = BOARD_PROFILES.get(board_id, {})
    if "modules" not in p:
        p["modules"] = {}

    if modules is not None:
        p["modules"] = dict(modules)

    rgb = p.get("rgb", {})
    if rgb_pin is not None:
        rgb["pin"] = int(rgb_pin)
    if rgb_n is not None:
        rgb["n"] = int(rgb_n)
    if rgb_order is not None:
        rgb["order"] = str(rgb_order).upper()
    if rgb:
        p["rgb"] = rgb

    for k, v in extras.items():
        p[k] = v

    BOARD_PROFILES[board_id] = p

def _apply_order(rgb, order):
    r, g, b = rgb
    order = (order or "GRB").upper()
    mp = {"R": r, "G": g, "B": b}
    try:
        return (mp[order[0]], mp[order[1]], mp[order[2]])
    except Exception:
        return (g, r, b)  # fallback GRB

def _clamp8(v):
    try:
        v = int(v)
    except Exception:
        v = 0
    if v < 0: v = 0
    if v > 255: v = 255
    return v

def make_color(r, g, b):
    """
    Tạo màu dạng chuỗi "#RRGGBB" (lowercase).
    Dùng để truyền vào show()/blink_color()/rgb.on().
    """
    r = _clamp8(r); g = _clamp8(g); b = _clamp8(b)
    return "#{:02x}{:02x}{:02x}".format(r, g, b)

class OnboardRGB:
    """
    Điều khiển RGB onboard dạng NeoPixel/WS2812.
    - on/off
    - preset colors
    - set rgb
    - set hex "#RRGGBB"
    - blink (blocking)
    """
    def __init__(self, pin=None, n=None, order=None, board_id=None, auto=True):
        self.board_id = board_id or (detect_board_id() if auto else "UNKNOWN")
        self._pin_no = None
        self._n = 1
        self._order = "GRB"
        self._np = None
        self._last = COLORS["WHITE"]
        self._brightness = 1.0  # 0..1

        if Pin is None or _neopixel is None:
            return

        if pin is None or n is None or order is None:
            prof = get_profile(self.board_id) or {}
            rgbp = prof.get("rgb") or {}
            if pin is None:
                pin = rgbp.get("pin", None)
            if n is None:
                n = rgbp.get("n", 1)
            if order is None:
                order = rgbp.get("order", "GRB")

        if pin is None:
            return

        self._pin_no = int(pin)
        self._n = max(1, int(n or 1))
        self._order = str(order or "GRB").upper()

        try:
            self._np = _neopixel.NeoPixel(Pin(self._pin_no, Pin.OUT), self._n)
            self.off()
        except Exception:
            self._np = None

    @property
    def available(self):
        return self._np is not None

    @property
    def pin(self):
        return self._pin_no

    @property
    def n(self):
        return self._n

    @property
    def order(self):
        return self._order

    def brightness(self, value=None):
        if value is None:
            return self._brightness
        try:
            v = float(value)
        except Exception:
            return self._brightness
        if v < 0: v = 0.0
        if v > 1: v = 1.0
        self._brightness = v
        return self._brightness

    def _scale(self, rgb):
        b = self._brightness
        if b >= 0.999:
            return rgb
        r, g, bl = rgb
        return (int(r * b), int(g * b), int(bl * b))

    def _write_all(self, rgb):
        if not self._np:
            return
        rgb = self._scale(rgb)
        data = _apply_order(rgb, self._order)
        try:
            for i in range(self._n):
                self._np[i] = data
            self._np.write()
        except Exception:
            pass

    def off(self):
        self._write_all((0, 0, 0))

    def on(self, color=None):
        if color is None:
            rgb = self._last
        else:
            rgb = self._parse_color(color)
            self._last = rgb
        self._write_all(rgb)

    def fill(self, color):
        rgb = self._parse_color(color)
        self._last = rgb
        self._write_all(rgb)

    def rgb(self, r, g, b):
        rgb = (_clamp8(r), _clamp8(g), _clamp8(b))
        self._last = rgb
        self._write_all(rgb)

    def hex(self, hex_color):
        """
        Set theo màu "#RRGGBB" đã tạo từ make_color()
        """
        self.on(hex_color)

    def color(self, name):
        rgb = self._parse_color(name)
        self._last = rgb
        self._write_all(rgb)

    def blink(self, times=3, period_ms=250, color="WHITE", end_off=True):
        if not self._np:
            return
        times = int(times) if times is not None else 1
        if times < 1:
            return
        period_ms = max(20, int(period_ms))
        on_ms = max(1, period_ms // 2)
        off_ms = max(1, period_ms - on_ms)

        rgb = self._parse_color(color)
        self._last = rgb

        for _ in range(times):
            self._write_all(rgb)
            _sleep_ms(on_ms)
            self.off()
            _sleep_ms(off_ms)

        if not end_off:
            self._write_all(rgb)

    def _parse_color(self, c):
        if isinstance(c, tuple) or isinstance(c, list):
            if len(c) >= 3:
                return (_clamp8(c[0]), _clamp8(c[1]), _clamp8(c[2]))
            return (0, 0, 0)

        s = str(c).strip()
        su = s.upper()

        if su in COLORS:
            return COLORS[su]

        if s.startswith("#") and len(s) == 7:
            try:
                r = int(s[1:3], 16)
                g = int(s[3:5], 16)
                b = int(s[5:7], 16)
                return (_clamp8(r), _clamp8(g), _clamp8(b))
            except Exception:
                return (0, 0, 0)

        return COLORS["WHITE"]

    def info(self):
        return {
            "board_id": self.board_id,
            "machine": _machine_str(),
            "pin": self._pin_no,
            "n": self._n,
            "order": self._order,
            "available": self.available,
            "neopixel": _neopixel is not None,
        }

# =========================
# Singleton tiện dùng
# =========================
rgb = OnboardRGB(auto=True)

# --- Basic convenience ---
def on(color="WHITE"):
    rgb.on(color)

def off():
    rgb.off()

def color(name):
    rgb.color(name)

def set_rgb(r, g, b):
    rgb.rgb(r, g, b)

# --- New requested helpers ---
def show(c):
    """
    Sáng LED theo màu đã tạo (ví dụ: c = make_color(...))
    c có thể là "#RRGGBB" / (r,g,b) / tên màu preset
    """
    rgb.on(c)

def show_rgb(r, g, b):
    rgb.rgb(r, g, b)

def blink_color(c, period_ms=250, times=3, end_off=True):
    """
    Nháy LED theo màu tuỳ chọn (c: "#RRGGBB" hoặc tuple hoặc preset name)
    period_ms: chu kỳ nháy (ms)
    """
    rgb.blink(times=times, period_ms=period_ms, color=c, end_off=end_off)

# Backward compatible wrapper
def blink(times=3, period_ms=250, color_name="WHITE", end_off=True):
    rgb.blink(times=times, period_ms=period_ms, color=color_name, end_off=end_off)

def info():
    return rgb.info()
