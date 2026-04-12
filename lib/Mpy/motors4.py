from machine import Pin, PWM
from time import sleep_ms


class Motors4:
    """
    MEBlock-style MicroPython library for 4 PWM motors (one-way).

    Public API kept as requested:
    - begin()
    - deinit()
    - set_speed(motor, speed)
    - stop(motor)
    - stop_all()
    - set_all(a, b, c, d)
    - forward_all(speed)
    - run_all_for_ms(a, b, c, d, ms)
    - forward_all_for_ms(speed, ms)
    - ramp_all_for(a, b, c, d, step=5, step_ms=30, ms=1000)

    Default pin mapping:
        motor 1 / A -> pin 5
        motor 2 / B -> pin 3
        motor 3 / C -> pin 1
        motor 4 / D -> pin 7

    Notes:
    - Speed range: 0..100
    - Negative values are clamped to 0
    - This is normal one-way PWM control only
    """

    DEFAULT_PINS = (5, 3, 1, 7)

    def __init__(self, pins=DEFAULT_PINS, freq=20000, res_bits=10):
        if len(pins) != 4:
            raise ValueError("pins must have exactly 4 items")

        self._pins = tuple(int(p) for p in pins)
        self._freq = int(freq)
        self._res_bits = max(1, int(res_bits))
        self._logical_max = 65535 if self._res_bits >= 16 else ((1 << self._res_bits) - 1)

        self._pwms = [None, None, None, None]
        self._use_u16 = False
        self._hw_max = 1023
        self._inited = False
        self._speed = [0, 0, 0, 0]

    # -------------------------
    # init / deinit
    # -------------------------
    def begin(self):
        self.deinit()

        for i, pin_no in enumerate(self._pins):
            pwm = PWM(Pin(pin_no))
            try:
                pwm.freq(self._freq)
            except Exception:
                pass
            self._pwms[i] = pwm

        probe = self._pwms[0]
        if hasattr(probe, "duty_u16"):
            self._use_u16 = True
            self._hw_max = 65535
        elif hasattr(probe, "duty"):
            self._use_u16 = False
            self._hw_max = 1023
        else:
            self.deinit()
            raise RuntimeError("PWM backend does not support duty_u16() or duty().")

        self._inited = True
        self.stop_all()
        return True

    def deinit(self):
        for pwm in self._pwms:
            if pwm is not None:
                try:
                    pwm.deinit()
                except Exception:
                    pass

        self._pwms = [None, None, None, None]
        self._inited = False

    # -------------------------
    # helpers
    # -------------------------
    def _motor_index(self, motor):
        if isinstance(motor, str):
            m = motor.strip().upper()
            if m == "A":
                return 0
            if m == "B":
                return 1
            if m == "C":
                return 2
            if m == "D":
                return 3
            raise ValueError("motor must be 1..4 or A/B/C/D")

        motor = int(motor)
        if motor < 1 or motor > 4:
            raise ValueError("motor must be in range 1..4")
        return motor - 1

    def _clamp_speed(self, speed):
        speed = int(speed)
        if speed < 0:
            return 0
        if speed > 100:
            return 100
        return speed

    def _logical_to_hw(self, logical_duty):
        logical_duty = int(logical_duty)
        if logical_duty < 0:
            logical_duty = 0
        if logical_duty > self._logical_max:
            logical_duty = self._logical_max
        return (logical_duty * self._hw_max) // self._logical_max if self._logical_max else 0

    def _speed_to_logical_duty(self, speed):
        speed = self._clamp_speed(speed)
        return (speed * self._logical_max) // 100

    def _write_motor(self, idx, speed):
        if not self._inited:
            raise RuntimeError("Motors4.begin() has not been called")

        logical_duty = self._speed_to_logical_duty(speed)
        hw_duty = self._logical_to_hw(logical_duty)
        pwm = self._pwms[idx]

        if self._use_u16:
            pwm.duty_u16(hw_duty)
        else:
            pwm.duty(hw_duty)

        self._speed[idx] = self._clamp_speed(speed)

    def _ramp_all_to(self, target1, target2, target3, target4, step=5, step_ms=30):
        targets = [
            self._clamp_speed(target1),
            self._clamp_speed(target2),
            self._clamp_speed(target3),
            self._clamp_speed(target4),
        ]
        currents = [self._speed[0], self._speed[1], self._speed[2], self._speed[3]]

        step = max(1, abs(int(step)))
        step_ms = max(0, int(step_ms))

        done = False
        while not done:
            done = True

            for i in range(4):
                if currents[i] < targets[i]:
                    currents[i] += step
                    if currents[i] > targets[i]:
                        currents[i] = targets[i]
                elif currents[i] > targets[i]:
                    currents[i] -= step
                    if currents[i] < targets[i]:
                        currents[i] = targets[i]

                if currents[i] != targets[i]:
                    done = False

            self.set_all(currents[0], currents[1], currents[2], currents[3])
            if step_ms:
                sleep_ms(step_ms)

    # -------------------------
    # public API
    # -------------------------
    def set_speed(self, motor, speed):
        self._write_motor(self._motor_index(motor), speed)

    def stop(self, motor):
        self._write_motor(self._motor_index(motor), 0)

    def stop_all(self):
        self._write_motor(0, 0)
        self._write_motor(1, 0)
        self._write_motor(2, 0)
        self._write_motor(3, 0)

    def set_all(self, a, b, c, d):
        self._write_motor(0, a)
        self._write_motor(1, b)
        self._write_motor(2, c)
        self._write_motor(3, d)

    def forward_all(self, speed):
        speed = self._clamp_speed(speed)
        self.set_all(speed, speed, speed, speed)

    def run_all_for_ms(self, a, b, c, d, ms):
        self.set_all(a, b, c, d)
        sleep_ms(max(0, int(ms)))
        self.stop_all()

    def forward_all_for_ms(self, speed, ms):
        self.forward_all(speed)
        sleep_ms(max(0, int(ms)))
        self.stop_all()

    def ramp_all_for(self, a, b, c, d, step=5, step_ms=30, ms=1000):
        """
        Ramp all motors from current speeds to targets (a,b,c,d),
        then keep those speeds for ms milliseconds, then stop all.
        """
        self._ramp_all_to(a, b, c, d, step=step, step_ms=step_ms)
        sleep_ms(max(0, int(ms)))
        self.stop_all()
