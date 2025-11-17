# onboard.py
# Thư viện thiết bị tích hợp trên ESP32-S3 DevKitC-1
#
# Tiến độ:
#   - OnboardRGB: điều khiển LED RGB WS2812 tích hợp trên board
#
# LED RGB ONBOARD:
#   - Loại WS2812 (NeoPixel)
#   - Nối vào GPIO48
#
# Ví dụ:
#   from onboard import OnboardRGB, RED, GREEN, BLUE
#
#   led = OnboardRGB()          # dùng preset: pin=48, 1 LED
#   led.red()                   # đỏ
#   led.green(128)              # xanh lá mờ
#   led.color(RED | BLUE, 200)  # tím
#   led.off()                   # tắt
#
#   # hoặc dùng singleton tiện lợi:
#   from onboard import get_rgb
#   led = get_rgb()
#   led.white(128)

from machine import Pin
import neopixel

# ========== THÔNG TIN BOARD / PRESET CHÂN ==========

BOARD_NAME = "ESP32-S3 DevKitC-1"

# LED RGB onboard: WS2812 nối vào GPIO48, chỉ có 1 bóng
S3_ONBOARD_RGB_PIN   = 48
S3_ONBOARD_RGB_NLEDS = 1


# ========== ĐỊNH NGHĨA MÀU CƠ BẢN (BITMASK) ==========

RED   = 0x01
GREEN = 0x02
BLUE  = 0x04


class OnboardRGB:
    """
    Điều khiển LED RGB (WS2812) tích hợp trên ESP32-S3 DevKitC-1.

    - Mặc định:
        pin       : 48
        nled      : 1
        brightness: 0.0–1.0 (hệ số nhân độ sáng)

    API chính:
        set_rgb(r, g, b)          : đặt màu trực tiếp (0–255)
        get_rgb()                 : trả về (r, g, b) logic (chưa nhân brightness)
        set_brightness(value)     : 0.0–1.0
        color(mask, brightness)   : RED | GREEN | BLUE
        off()
        red()/green()/blue()/white()
        blink(mask, brightness, on_ms, off_ms, times)
    """

    def __init__(self, pin=S3_ONBOARD_RGB_PIN,
                 nled=S3_ONBOARD_RGB_NLEDS,
                 brightness=1.0,
                 auto_show=True):
        self.pin_num = int(pin)
        self.n = int(nled) if nled is not None else 1
        if self.n <= 0:
            raise ValueError("nled must be >= 1")

        self.auto_show = bool(auto_show)
        self._brightness = 1.0
        self._color = (0, 0, 0)

        self.pin = Pin(self.pin_num, Pin.OUT)
        self.np = neopixel.NeoPixel(self.pin, self.n)

        self.set_brightness(brightness, show=False)
        self._apply_color(show=True)

    # ------- nội bộ -------

    @staticmethod
    def _clamp255(v):
        return max(0, min(255, int(v)))

    def _scaled_color(self):
        """Áp brightness lên màu logic hiện tại."""
        br = self._brightness
        r, g, b = self._color
        return int(r * br), int(g * br), int(b * br)

    def _apply_color(self, show=True):
        r, g, b = self._scaled_color()
        for i in range(self.n):
            self.np[i] = (r, g, b)
        if show and self.auto_show:
            self.np.write()

    # ------- API công khai -------

    def set_rgb(self, r, g, b, show=True):
        """Đặt màu trực tiếp (0–255)."""
        self._color = (
            self._clamp255(r),
            self._clamp255(g),
            self._clamp255(b),
        )
        self._apply_color(show=show)

    def get_rgb(self):
        """Trả về màu logic hiện tại (0–255, chưa nhân brightness)."""
        return self._color

    def set_brightness(self, value, show=True):
        """Đặt độ sáng chung (0.0–1.0)."""
        v = float(value)
        if v < 0:
            v = 0.0
        if v > 1:
            v = 1.0
        self._brightness = v
        self._apply_color(show=show)

    def off(self, show=True):
        """Tắt LED (hiển thị 0,0,0 nhưng vẫn giữ màu logic)."""
        old_color = self._color
        self._color = (0, 0, 0)
        self._apply_color(show=show)
        self._color = old_color

    def color(self, mask, brightness=None, show=True):
        """
        Đặt màu bằng bitmask RED|GREEN|BLUE và độ sáng 0–255.

        Ví dụ:
            led.color(RED)                # đỏ
            led.color(RED | BLUE)         # tím
            led.color(RED | GREEN, 180)   # vàng, sáng 180/255
        """
        if brightness is None:
            brightness = 255
        b = self._clamp255(brightness)

        r = b if (mask & RED)   else 0
        g = b if (mask & GREEN) else 0
        bl = b if (mask & BLUE) else 0
        self.set_rgb(r, g, bl, show=show)

    # ------- các màu shortcut -------

    def white(self, brightness=255, show=True):
        self.color(RED | GREEN | BLUE, brightness, show=show)

    def red(self, brightness=255, show=True):
        self.color(RED, brightness, show=show)

    def green(self, brightness=255, show=True):
        self.color(GREEN, brightness, show=show)

    def blue(self, brightness=255, show=True):
        self.color(BLUE, brightness, show=show)

    # ------- hiệu ứng đơn giản -------

    def blink(self, mask, brightness=255,
              on_ms=200, off_ms=200, times=3):
        """Nháy LED blocking với màu từ bitmask."""
        import time
        for _ in range(int(times)):
            self.color(mask, brightness, show=True)
            time.sleep_ms(on_ms)
            self.off(show=True)
            time.sleep_ms(off_ms)


# ========== SINGLETON TIỆN DÙNG ==========

_rgb_singleton = None

def get_rgb():
    """
    Trả về 1 instance dùng chung của OnboardRGB.
    Lần gọi đầu sẽ khởi tạo, các lần sau dùng lại.
    """
    global _rgb_singleton
    if _rgb_singleton is None:
        _rgb_singleton = OnboardRGB()
    return _rgb_singleton
