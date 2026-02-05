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
#   servo_180(pin, angle, speed=0..100)
#   servo_270(pin, angle, speed=0..100)
#   servo_360(pin, speed_percent)
#   servo_off(pin)
#
# + Alias mới (positional speed):
#   servo_180_to(pin, angle, speed)
#   servo_270_to(pin, angle, speed)
#
# pin : có thể là số GPIO (0, 1, 2, ...) hoặc machine.Pin
#
import time
from machine import Pin, PWM

_channels = {}
_last_angles = {}

def _clamp(v, vmin, vmax):
    if v < vmin:
        return vmin
    if v > vmax:
        return vmax
    return v

def _key_from_pin(pin):
    if isinstance(pin, Pin):
        try:
            return ("pin", pin.id())
        except Exception:
            return ("pin_obj", id(pin))
    return ("num", int(pin))

class _ServoChannel:
    def __init__(self, pin, freq=50):
        if isinstance(pin, Pin):
            self.pin = pin
        else:
            self.pin = Pin(int(pin), Pin.OUT)
        self.freq = int(freq)
        self.pwm = PWM(self.pin, freq=self.freq)

    def _pulse_to_duty_u16(self, pulse_us):
        period_us = 1_000_000.0 / self.freq
        duty_fraction = pulse_us / period_us
        duty_u16 = int(65535 * duty_fraction)
        return _clamp(duty_u16, 0, 65535)

    def write_pulse(self, pulse_us):
        duty_u16 = self._pulse_to_duty_u16(pulse_us)
        if hasattr(self.pwm, "duty_u16"):
            self.pwm.duty_u16(duty_u16)
        else:
            self.pwm.duty(duty_u16 >> 6)

    def off(self):
        if hasattr(self.pwm, "duty_u16"):
            self.pwm.duty_u16(0)
        else:
            self.pwm.duty(0)
        self.pwm.deinit()

def _get_channel(pin, freq=50):
    key = _key_from_pin(pin)
    ch = _channels.get(key)
    if ch is None:
        ch = _ServoChannel(pin, freq=freq)
        _channels[key] = ch
    return ch

def _angle_to_pulse(angle, angle_range, min_us=500, max_us=2500):
    a = _clamp(float(angle), 0.0, float(angle_range))
    span = max_us - min_us
    return min_us + span * (a / float(angle_range))

def _move_to_angle(
    pin,
    target_angle,
    angle_range,
    speed,
    *,
    smooth_deg=0.5,
    freq=50,
    min_us=500,
    max_us=2500,
):
    """
    speed: 0..100 (100=nhảy ngay, 0=chậm nhất)
    smooth_deg: mặc định 0.5° (độ mượt)
    """
    sp = int(_clamp(float(speed), 0.0, 100.0))
    if sp <= 0:
        return  # speed 0: không xoay
    target = _clamp(float(target_angle), 0.0, float(angle_range))

    ch = _get_channel(pin, freq=freq)
    state_key = (_key_from_pin(pin), int(angle_range))
    cur = _last_angles.get(state_key)

    if cur is None or sp >= 100:
        ch.write_pulse(_angle_to_pulse(target, angle_range, min_us=min_us, max_us=max_us))
        _last_angles[state_key] = target
        return

    cur = _clamp(float(cur), 0.0, float(angle_range))

    step_deg = float(smooth_deg)
    if step_deg <= 0:
        step_deg = 0.5
    if step_deg < 0.1:
        step_deg = 0.1

    diff = target - cur
    if abs(diff) < step_deg:
        ch.write_pulse(_angle_to_pulse(target, angle_range, min_us=min_us, max_us=max_us))
        _last_angles[state_key] = target
        return

    base_ms_per_deg = (100 - sp) * 20.0 / 100.0
    delay_ms = int(round(base_ms_per_deg * step_deg))
    if delay_ms <= 0:
        ch.write_pulse(_angle_to_pulse(target, angle_range, min_us=min_us, max_us=max_us))
        _last_angles[state_key] = target
        return

    step = step_deg if diff > 0 else -step_deg
    steps = int(abs(diff) / step_deg)

    a = cur
    for _ in range(steps):
        a += step
        if a < 0:
            a = 0.0
        if a > float(angle_range):
            a = float(angle_range)
        ch.write_pulse(_angle_to_pulse(a, angle_range, min_us=min_us, max_us=max_us))
        time.sleep_ms(delay_ms)

    ch.write_pulse(_angle_to_pulse(target, angle_range, min_us=min_us, max_us=max_us))
    _last_angles[state_key] = target

def servo_180(pin, angle, *, speed=100, smooth_deg=0.5, freq=50, min_us=500, max_us=2500):
    _move_to_angle(pin, angle, 180, speed, smooth_deg=smooth_deg, freq=freq, min_us=min_us, max_us=max_us)

def servo_270(pin, angle, *, speed=100, smooth_deg=0.5, freq=50, min_us=500, max_us=2500):
    _move_to_angle(pin, angle, 270, speed, smooth_deg=smooth_deg, freq=freq, min_us=min_us, max_us=max_us)

# Alias positional speed (dễ generate block)
def servo_180_to(pin, angle, speed=100, *, smooth_deg=0.5, freq=50, min_us=500, max_us=2500):
    _move_to_angle(pin, angle, 180, speed, smooth_deg=smooth_deg, freq=freq, min_us=min_us, max_us=max_us)

def servo_270_to(pin, angle, speed=100, *, smooth_deg=0.5, freq=50, min_us=500, max_us=2500):
    _move_to_angle(pin, angle, 270, speed, smooth_deg=smooth_deg, freq=freq, min_us=min_us, max_us=max_us)

def _speed_percent_to_pulse(percent, min_us=1000, max_us=2000, center_offset_us=0):
    p = _clamp(float(percent), -100.0, 100.0)
    center = (min_us + max_us) / 2.0 + center_offset_us
    span = (max_us - min_us) / 2.0
    return center + span * (p / 100.0)

def servo_360(pin, speed_percent, *, freq=50, min_us=1000, max_us=2000, center_offset_us=0):
    ch = _get_channel(pin, freq=freq)
    ch.write_pulse(_speed_percent_to_pulse(speed_percent, min_us=min_us, max_us=max_us, center_offset_us=center_offset_us))

def servo_off(pin):
    key = _key_from_pin(pin)
    ch = _channels.pop(key, None)
    if ch is not None:
        ch.off()
    for r in (180, 270):
        _last_angles.pop((key, r), None)
