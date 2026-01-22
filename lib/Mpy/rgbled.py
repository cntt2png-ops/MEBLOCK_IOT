from machine import Pin, PWM
# rgbled.py – Especially for ESP32-S3 DevKitC-1 (preset)

RED   = 0x01
GREEN = 0x02
BLUE  = 0x04

# ===== PRESET CHÂN RGB CHO ESP32-S3 DEVKITC-1 =====
# Gợi ý:
#   - R -> GPIO5
#   - G -> GPIO6
#   - B -> GPIO7
S3_RGB_R = 5
S3_RGB_G = 6
S3_RGB_B = 7


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

    Nếu không truyền pin_r / pin_g / pin_b:
        - pin_r = S3_RGB_R
        - pin_g = S3_RGB_G
        - pin_b = S3_RGB_B
    """

    def __init__(self, pin_r=None, pin_g=None, pin_b=None,
                 common_anode=False, freq=1000):
        # Nếu không truyền -> dùng preset S3
        if pin_r is None:
            pin_r = S3_RGB_R
        if pin_g is None:
            pin_g = S3_RGB_G
        if pin_b is None:
            pin_b = S3_RGB_B

        self.red_pwm = PWM(Pin(pin_r), freq=freq)
        self.green_pwm = PWM(Pin(pin_g), freq=freq)
        self.blue_pwm = PWM(Pin(pin_b), freq=freq)

        # Trên ESP32/ESP32-S3, độ phân giải thường là 10 bit (0–1023)
        self.max_duty = 1023
        self.common_anode = common_anode

        # Lưu giá trị hiện tại (0–255)
        self._r = 0
        self._g = 0
        self._b = 0

        self._update_pwm()

    def _scale_0_255_to_duty(self, val):
        # Giới hạn 0–255
        val = max(0, min(255, int(val)))
        duty = int(val * self.max_duty / 255)

        if self.common_anode:
            duty = self.max_duty - duty
        return duty

    def _update_pwm(self):
        self.red_pwm.duty(self._scale_0_255_to_duty(self._r))
        self.green_pwm.duty(self._scale_0_255_to_duty(self._g))
        self.blue_pwm.duty(self._scale_0_255_to_duty(self._b))

    # ===== API màu cơ bản =====

    def set_rgb(self, r, g, b):
        """
        Đặt màu trực tiếp (0–255).
        """
        self._r = max(0, min(255, int(r)))
        self._g = max(0, min(255, int(g)))
        self._b = max(0, min(255, int(b)))
        self._update_pwm()

    def get_rgb(self):
        """
        Trả về (r, g, b) 0–255.
        """
        return (self._r, self._g, self._b)

    def export_rgb(self, sep=" "):
        """
        Trả về chuỗi "r<sep>g<sep>b", ví dụ "0 255 255".
        """
        return f"{self._r}{sep}{self._g}{sep}{self._b}"

    def off(self):
        """
        Tắt LED.
        """
        self.set_rgb(0, 0, 0)

    def color(self, mask, brightness=255):
        """
        mask: kết hợp RED | GREEN | BLUE.
        brightness: 0–255.
        Ví dụ:
            led.color(RED)             -> chỉ đỏ
            led.color(RED | BLUE)      -> tím
            led.color(RED | GREEN)     -> vàng
            led.color(RED | GREEN | BLUE) -> trắng
        """
        b = max(0, min(255, int(brightness)))
        r = b if (mask & RED)   else 0
        g = b if (mask & GREEN) else 0
        bl = b if (mask & BLUE) else 0
        self.set_rgb(r, g, bl)

    def white(self, brightness=255):
        self.color(RED | GREEN | BLUE, brightness)

    def yellow(self, brightness=255):
        self.color(RED | GREEN, brightness)

    def cyan(self, brightness=255):
        self.color(GREEN | BLUE, brightness)

    def magenta(self, brightness=255):
        self.color(RED | BLUE, brightness)
