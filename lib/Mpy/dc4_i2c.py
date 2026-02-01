# dc4_i2c_tb6612.py
# Control 4 DC motors via I2C PCA9685 -> TB6612FNG (2 chips)
# API:
#   m = DC4Motor(i2c, addr=0x40, pwm_freq=1000)
#   m.set(1, 60)     # M1 forward 60%
#   m.set(2, -30)    # M2 reverse 30%
#   m.stop(3)        # coast
#   m.stop(4, brake=True)
#   m.stop_all()

from micropython import const
import time

# PCA9685 registers
_MODE1      = const(0x00)
_MODE2      = const(0x01)
_PRESCALE   = const(0xFE)
_LED0_ON_L  = const(0x06)

_RESTART    = const(0x80)
_SLEEP      = const(0x10)
_AI         = const(0x20)   # auto-increment

# MODE2 bits
_OUTDRV     = const(0x04)   # totem pole


class PCA9685:
    def __init__(self, i2c, addr=0x40):
        self.i2c = i2c
        self.addr = addr
        self.reset()

    def _w8(self, reg, val):
        self.i2c.writeto_mem(self.addr, reg, bytes([val & 0xFF]))

    def _r8(self, reg):
        return self.i2c.readfrom_mem(self.addr, reg, 1)[0]

    def reset(self):
        # MODE1: AI on, normal
        self._w8(_MODE1, _AI)
        # MODE2: totem pole (recommended)
        self._w8(_MODE2, _OUTDRV)
        time.sleep_ms(10)

    def set_pwm_freq(self, freq_hz):
        # prescale = round(25MHz/(4096*freq)) - 1
        # Limit PCA9685 practical freq ~ 24..1526Hz
        if freq_hz < 24:
            freq_hz = 24
        if freq_hz > 1526:
            freq_hz = 1526

        prescale = int((25000000.0 / (4096.0 * float(freq_hz))) + 0.5) - 1
        if prescale < 3:
            prescale = 3
        if prescale > 255:
            prescale = 255

        old = self._r8(_MODE1)
        self._w8(_MODE1, (old & ~_RESTART) | _SLEEP)  # sleep
        self._w8(_PRESCALE, prescale)
        self._w8(_MODE1, old)                          # wake
        time.sleep_ms(5)
        self._w8(_MODE1, old | _RESTART | _AI)         # restart + AI

    def set_pwm(self, ch, on, off):
        base = _LED0_ON_L + 4 * int(ch)
        self.i2c.writeto_mem(
            self.addr,
            base,
            bytes([
                on & 0xFF, (on >> 8) & 0xFF,
                off & 0xFF, (off >> 8) & 0xFF
            ])
        )

    def set_level(self, ch, level):
        # Full ON / Full OFF using bit 12
        if level:
            self.set_pwm(ch, 4096, 0)   # full on
        else:
            self.set_pwm(ch, 0, 4096)   # full off

    def set_duty(self, ch, duty_0_4095):
        d = int(duty_0_4095)
        if d <= 0:
            self.set_level(ch, 0)
        elif d >= 4095:
            self.set_level(ch, 1)
        else:
            self.set_pwm(ch, 0, d)


class DC4Motor:
    """
    Speed: -100..100
      >0: forward, <0: reverse
    stop(brake=False):
      brake=False -> coast (IN1=0 IN2=0, PWM=0)
      brake=True  -> short brake (IN1=1 IN2=1, PWM=100%)
    """

    def __init__(self, i2c, addr=0x40, pwm_freq=1000, debug=False):
        self.debug = debug
        self.pca = PCA9685(i2c, addr=addr)
        self.pca.set_pwm_freq(pwm_freq)

        # Motor map theo sơ đồ bạn gửi:
        # Each: (PWM, IN1, IN2)
        self._mot = {
            1: (0, 2, 1),     # M1: PWM=LED0, IN1=LED2, IN2=LED1  (đúng)
            2: (3, 4, 5),     # M2: PWM=LED3, IN1=LED4, IN2=LED5  (đúng)
            3: (6, 7, 8),     # M3: PWM=LED6, IN1=LED7(IN31), IN2=LED8(IN32)  (sửa)
            4: (11, 9, 10),   # M4: PWM=LED11(PWMB4), IN1=LED9(INB41), IN2=LED10(INB42) (sửa)
        }


        # Allow invert per motor if wiring reversed
        self._invert = {1: False, 2: False, 3: False, 4: False}

        # Init all motors off
        self.stop_all(brake=False)

    def invert(self, motor, enable=True):
        self._invert[int(motor)] = bool(enable)

    def _set_dir(self, in1, in2, forward):
        # forward True: IN1=1 IN2=0 ; reverse: IN1=0 IN2=1
        if forward:
            self.pca.set_level(in1, 1)
            self.pca.set_level(in2, 0)
        else:
            self.pca.set_level(in1, 0)
            self.pca.set_level(in2, 1)

    def set(self, motor, speed):
        m = int(motor)
        if m not in self._mot:
            raise ValueError("motor must be 1..4")

        pwm, in1, in2 = self._mot[m]

        s = int(speed)
        if s > 100: s = 100
        if s < -100: s = -100

        if s == 0:
            self.stop(m, brake=False)
            return

        forward = (s > 0)
        if self._invert[m]:
            forward = not forward

        duty = (abs(s) * 4095) // 100

        # set direction first, then PWM
        self._set_dir(in1, in2, forward)
        self.pca.set_duty(pwm, duty)

    def stop(self, motor, brake=False):
        m = int(motor)
        if m not in self._mot:
            raise ValueError("motor must be 1..4")

        pwm, in1, in2 = self._mot[m]

        if brake:
            # short brake: IN1=1 IN2=1, PWM=100%
            self.pca.set_level(in1, 1)
            self.pca.set_level(in2, 1)
            self.pca.set_duty(pwm, 4095)
        else:
            # coast: IN1=0 IN2=0, PWM=0
            self.pca.set_level(in1, 0)
            self.pca.set_level(in2, 0)
            self.pca.set_duty(pwm, 0)

    def stop_all(self, brake=False):
        for m in (1, 2, 3, 4):
            self.stop(m, brake=brake)
