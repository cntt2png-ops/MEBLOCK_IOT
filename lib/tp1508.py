from machine import Pin, PWM
from time import sleep

# ==== CHÂN ĐIỀU KHIỂN (CHỈNH LẠI THEO BOARD CỦA BẠN) ====
IN1_PIN = 1   # Motor A
IN2_PIN = 2   # Motor A

IN3_PIN = 3  # Motor B
IN4_PIN = 4  # Motor B

MAX_DUTY = 65535


class Motor:
    def __init__(self, in1_pin, in2_pin, freq=1000):
        self.pwm1 = PWM(Pin(in1_pin, Pin.OUT), freq=freq)
        self.pwm2 = PWM(Pin(in2_pin, Pin.OUT), freq=freq)
        self.stop()

    def _speed_to_duty(self, speed):
        # speed: -1.0 .. 1.0
        if speed > 1:
            speed = 1
        if speed < -1:
            speed = -1
        return int(abs(speed) * MAX_DUTY)

    def set_speed(self, speed):
        """
        speed:
          > 0: quay thuận
          < 0: quay ngược
          = 0: dừng
        """
        if speed == 0:
            self.stop()
            return

        duty = self._speed_to_duty(speed)

        if speed > 0:
            # forward: IN1 = PWM, IN2 = 0
            self.pwm1.duty_u16(duty)
            self.pwm2.duty_u16(0)
        else:
            # backward: IN1 = 0, IN2 = PWM
            self.pwm1.duty_u16(0)
            self.pwm2.duty_u16(duty)

    def stop(self):
        # thả nổi (coast)
        self.pwm1.duty_u16(0)
        self.pwm2.duty_u16(0)

    def brake(self):
        # phanh cứng: cả 2 HIGH
        self.pwm1.duty_u16(MAX_DUTY)
        self.pwm2.duty_u16(MAX_DUTY)