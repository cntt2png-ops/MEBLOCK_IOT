# servo.py
# Thư viện điều khiển servo cho ESP32 / ESP32-S3
#
# Hỗ trợ:
#   - Servo 180° : quay theo góc 0..180
#   - Servo 270° : quay theo góc 0..270
#   - Servo 360° (liên tục): quay theo tốc độ -100..100 (%)
#   - Tắt điều khiển servo trên 1 chân
#
# API chính:
#   servo_180(pin, angle)
#   servo_270(pin, angle)
#   servo_360(pin, speed_percent)
#   servo_off(pin)
#
# pin : có thể là số GPIO (0, 1, 2, ...) hoặc machine.Pin
#
# Ví dụ:
#   from servo import servo_180, servo_270, servo_360, servo_off
#   import time
#
#   # Servo 180°
#   servo_180(P0, 90)         # quay servo 180° ở chân P0 đến góc 90°
#
#   # Servo 270°
#   servo_270(P1, 135)        # quay servo 270° ở chân P1 đến góc 135°
#
#   # Servo 360° (liên tục)
#   servo_360(P2, 50)         # quay thuận 50%
#   time.sleep(2)
#   servo_360(P2, -100)       # quay ngược 100%
#   time.sleep(1)
#   servo_off(P2)             # tắt điều khiển servo chân P2
#

from machine import Pin, PWM

# Lưu PWM cho từng chân để tái sử dụng
_channels = {}


def _clamp(v, vmin, vmax):
    if v < vmin:
        return vmin
    if v > vmax:
        return vmax
    return v


def _key_from_pin(pin):
    """Tạo key cho dict từ pin (int hoặc Pin)."""
    if isinstance(pin, Pin):
        try:
            return ("pin", pin.id())
        except Exception:
            # fallback nếu Pin không có id()
            return ("pin_obj", id(pin))
    return ("num", int(pin))


class _ServoChannel:
    """Đại diện một kênh PWM điều khiển servo trên 1 chân."""

    def __init__(self, pin, freq=50):
        if isinstance(pin, Pin):
            self.pin = pin
        else:
            self.pin = Pin(int(pin), Pin.OUT)

        self.freq = int(freq)
        self.pwm = PWM(self.pin, freq=self.freq)

    def _pulse_to_duty_u16(self, pulse_us):
        """
        Chuyển độ rộng xung (us) sang duty_u16 (0..65535)
        dựa trên tần số PWM hiện tại.
        """
        period_us = 1_000_000.0 / self.freq  # chu kỳ (us)
        duty_fraction = pulse_us / period_us  # 0..1
        duty_u16 = int(65535 * duty_fraction)
        return _clamp(duty_u16, 0, 65535)

    def write_pulse(self, pulse_us):
        """Ghi xung (us) ra PWM."""
        duty_u16 = self._pulse_to_duty_u16(pulse_us)
        if hasattr(self.pwm, "duty_u16"):
            self.pwm.duty_u16(duty_u16)
        else:
            # fallback cho port chỉ hỗ trợ duty(0..1023)
            self.pwm.duty(duty_u16 >> 6)  # ~ /64

    def off(self):
        """Tắt PWM trên chân này."""
        if hasattr(self.pwm, "duty_u16"):
            self.pwm.duty_u16(0)
        else:
            self.pwm.duty(0)
        self.pwm.deinit()


def _get_channel(pin, freq=50):
    """Lấy (hoặc tạo) kênh PWM cho 1 pin."""
    key = _key_from_pin(pin)
    ch = _channels.get(key)
    if ch is None:
        ch = _ServoChannel(pin, freq=freq)
        _channels[key] = ch
    return ch


# =========================================================
#                 SERVO 180 & 270 ĐỘ
# =========================================================

def _angle_to_pulse(angle, angle_range, min_us=500, max_us=2500):
    """Map góc [0..angle_range] -> xung [min_us..max_us]."""
    a = _clamp(float(angle), 0.0, float(angle_range))
    span = max_us - min_us
    pulse = min_us + span * (a / float(angle_range))
    return pulse


def servo_180(pin, angle, *, freq=50, min_us=500, max_us=2500):
    """
    Quay servo 180° ở chân pin đến góc angle (0..180 độ).
    """
    ch = _get_channel(pin, freq=freq)
    pulse = _angle_to_pulse(angle, 180, min_us=min_us, max_us=max_us)
    ch.write_pulse(pulse)


def servo_270(pin, angle, *, freq=50, min_us=500, max_us=2500):
    """
    Quay servo 270° ở chân pin đến góc angle (0..270 độ).
    """
    ch = _get_channel(pin, freq=freq)
    pulse = _angle_to_pulse(angle, 270, min_us=min_us, max_us=max_us)
    ch.write_pulse(pulse)


# =========================================================
#            SERVO 360° (LIÊN TỤC) - TỐC ĐỘ -100..100
# =========================================================

def _speed_percent_to_pulse(percent, min_us=1000, max_us=2000, center_offset_us=0):
    """
    Map tốc độ phần trăm (-100..100) sang xung servo (us).

        -100 -> min_us   (quay ngược nhanh nhất)
          0 -> center    (dừng)
        100 -> max_us    (quay thuận nhanh nhất)
    """
    p = _clamp(float(percent), -100.0, 100.0)
    center = (min_us + max_us) / 2.0 + center_offset_us
    span = (max_us - min_us) / 2.0
    s = p / 100.0  # -1..1
    pulse = center + span * s
    return pulse


def servo_360(
    pin,
    speed_percent,
    *,
    freq=50,
    min_us=1000,
    max_us=2000,
    center_offset_us=0,
):
    """
    Điều khiển servo 360° (servo quay liên tục) theo tốc độ phần trăm:

        speed_percent: -100 .. 100
            -100 -> quay ngược max
              0 -> dừng
            100 -> quay thuận max
    """
    ch = _get_channel(pin, freq=freq)
    pulse = _speed_percent_to_pulse(
        speed_percent,
        min_us=min_us,
        max_us=max_us,
        center_offset_us=center_offset_us,
    )
    ch.write_pulse(pulse)


# =========================================================
#                      TẮT ĐIỀU KHIỂN
# =========================================================

def servo_off(pin):
    """
    x:
      - Duty = 0
      - Deinit PWM và xóa khỏi danh sách kênh
    """
    key = _key_from_pin(pin)
    ch = _channels.pop(key, None)
    if ch is not None:
        ch.off()
