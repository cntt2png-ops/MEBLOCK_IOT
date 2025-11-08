from machine import Pin, PWM

# ====== BITMASK MÀU CƠ BẢN ======
RED   = 0x01
GREEN = 0x02
BLUE  = 0x04


class RGBLed:
    """
    Driver LED RGB 4 chân cho ESP32/ESP32-S3.

    - Điều khiển bằng 3 PWM: R, G, B
    - Hỗ trợ:
        + LED common cathode (chân chung GND)
        + LED common anode   (chân chung 3V3)

    API chính:
        - set_rgb(r, g, b)         : đặt màu trực tiếp (0–255)
        - get_rgb()                : trả về (r, g, b)
        - export_rgb(sep=" ")      : trả về chuỗi "r<sep>g<sep>b" (vd: "0 255 255")
        - color(mask, brightness)  : dùng RED | GREEN | BLUE để trộn màu
    """

    def __init__(self, pin_r, pin_g, pin_b, common_anode=False, freq=1000):
        self.red_pwm = PWM(Pin(pin_r), freq=freq)
        self.green_pwm = PWM(Pin(pin_g), freq=freq)
        self.blue_pwm = PWM(Pin(pin_b), freq=freq)

        self.max_duty = 1023          # ESP32: 10-bit
        self.common_anode = common_anode

        # Trạng thái hiện tại (0–255)
        self._r = 0
        self._g = 0
        self._b = 0

        self._update_pwm()

    # ====== HÀM NỘI BỘ ======
    def _scale_0_255_to_duty(self, val):
        val = max(0, min(255, int(val)))
        duty = int(val * self.max_duty / 255)
        if self.common_anode:
            duty = self.max_duty - duty
        return duty

    def _update_pwm(self):
        self.red_pwm.duty(self._scale_0_255_to_duty(self._r))
        self.green_pwm.duty(self._scale_0_255_to_duty(self._g))
        self.blue_pwm.duty(self._scale_0_255_to_duty(self._b))

    # ====== RGB TRỰC TIẾP (0–255, 0–255, 0–255) ======
    def set_rgb(self, r, g, b):
        """Đặt màu trực tiếp theo RGB (mỗi kênh 0–255)."""
        self._r = max(0, min(255, int(r)))
        self._g = max(0, min(255, int(g)))
        self._b = max(0, min(255, int(b)))
        self._update_pwm()

    def get_rgb(self):
        """Trả về màu hiện tại dạng tuple (r, g, b)."""
        return (self._r, self._g, self._b)

    def export_rgb(self, sep=" "):
        """
        Export màu hiện tại thành chuỗi:
        - mặc định: "r g b" (vd: "0 255 255")
        - có thể đổi sep="," -> "0,255,255"
        """
        return f"{self._r}{sep}{self._g}{sep}{self._b}"

    def off(self):
        """Tắt LED."""
        self.set_rgb(0, 0, 0)

    # ====== API KIỂU BITMASK: color(RED | GREEN | BLUE) ======
    def color(self, mask, brightness=255):
        """
        Đặt màu bằng bitmask:

        Ví dụ:
            led.color(RED)
            led.color(GREEN)
            led.color(BLUE)
            led.color(RED | GREEN)         # vàng
            led.color(GREEN | BLUE)        # cyan
            led.color(RED | BLUE)          # magenta
            led.color(RED | GREEN | BLUE)  # trắng

        brightness: 0–255 (độ sáng chung của các kênh đang bật)
        """
        b = max(0, min(255, int(brightness)))

        r = b if (mask & RED)   else 0
        g = b if (mask & GREEN) else 0
        bl = b if (mask & BLUE) else 0

        self.set_rgb(r, g, bl)

    # ====== HÀM TIỆN DÙNG SẴN ======
    def white(self, brightness=255):
        self.color(RED | GREEN | BLUE, brightness)

    def yellow(self, brightness=255):
        self.color(RED | GREEN, brightness)

    def cyan(self, brightness=255):
        self.color(GREEN | BLUE, brightness)

    def magenta(self, brightness=255):
        self.color(RED | BLUE, brightness)
