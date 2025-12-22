from machine import Pin, PWM

# Dùng duty_u16: 0..65535
MAX_DUTY = 65535

# Motor name mặc định
MOTOR_A = "A"
MOTOR_B = "B"

# Lưu các motor theo tên: "A", "B"
_motors = {}


class MotorChannel:
    """
    Điều khiển 1 kênh motor DC dùng 2 chân PWM:
      - in1_pin, in2_pin: số GPIO
      - speed: -100..100 (%)
        >0  -> quay thuận
        <0  -> quay ngược
        =0  -> dừng
    """
    def __init__(self, in1_pin, in2_pin, freq=1000):
        self.pwm1 = PWM(Pin(in1_pin, Pin.OUT), freq=freq)
        self.pwm2 = PWM(Pin(in2_pin, Pin.OUT), freq=freq)
        self.stop()

    def _speed_to_duty(self, speed_percent):
        # Clamp -100..100
        if speed_percent > 100:
            speed_percent = 100
        if speed_percent < -100:
            speed_percent = -100
        duty = int(abs(speed_percent) / 100 * MAX_DUTY)
        return duty, speed_percent

    def set_speed_percent(self, speed_percent):
        """
        speed_percent: -100..100
        """
        if speed_percent == 0:
            self.stop()
            return

        duty, speed_percent = self._speed_to_duty(speed_percent)

        if speed_percent > 0:
            # forward: IN1 = PWM, IN2 = 0
            self.pwm1.duty_u16(duty)
            self.pwm2.duty_u16(0)
        else:
            # backward: IN1 = 0, IN2 = PWM
            self.pwm1.duty_u16(0)
            self.pwm2.duty_u16(duty)

    def stop(self):
        # Thả trôi (coast): cả 2 = 0
        self.pwm1.duty_u16(0)
        self.pwm2.duty_u16(0)

    def brake(self):
        # Phanh cứng: cả 2 HIGH
        self.pwm1.duty_u16(MAX_DUTY)
        self.pwm2.duty_u16(MAX_DUTY)


def setup_tc1508a(in1_pin, in2_pin, in3_pin=None, in4_pin=None, freq=1000):
    """
    Cấu hình mặc định driver TC1508A:
      - Motor A = (IN1, IN2)  -> 2 chân đầu
      - Motor B = (IN3, IN4)  -> 2 chân sau (nếu có)

    Ví dụ:
      setup_tc1508a(6, 7, 4, 5)
        -> Motor A dùng GPIO 6,7 (IN1,IN2)
        -> Motor B dùng GPIO 4,5 (IN3,IN4)

      Nếu chỉ có 1 motor:
      setup_tc1508a(6, 7)
        -> chỉ tạo Motor A
    """
    global _motors

    # Motor A: IN1, IN2
    _motors[MOTOR_A] = MotorChannel(in1_pin, in2_pin, freq=freq)

    # Nếu có truyền thêm in3, in4 thì tạo luôn Motor B
    if in3_pin is not None and in4_pin is not None:
        _motors[MOTOR_B] = MotorChannel(in3_pin, in4_pin, freq=freq)


def set_motor_speed(name, speed_percent):
    """
    Điều khiển motor theo tên:
      name: "A" hoặc "B"
      speed_percent: -100..100

    Ví dụ:
      set_motor_speed("A", 80)    # Motor A quay thuận nhanh
      set_motor_speed("B", -50)   # Motor B quay ngược chậm
      set_motor_speed("A", 0)     # Dừng Motor A
    """
    m_name = str(name).strip().upper()
    motor = _motors.get(m_name)
    if motor is None:
        print("Motor '{}' chưa được setup_tc1508a()".format(m_name))
        return
    motor.set_speed_percent(speed_percent)


def stop_motor(name):
    """
    Dừng 1 motor theo tên.
    Ví dụ:
      stop_motor("A")
    """
    m_name = str(name).strip().upper()
    motor = _motors.get(m_name)
    if motor is None:
        print("Motor '{}' chưa được setup_tc1508a()".format(m_name))
        return
    motor.stop()


def brake_motor(name):
    """
    Phanh cứng 1 motor theo tên.
    Ví dụ:
      brake_motor("B")
    """
    m_name = str(name).strip().upper()
    motor = _motors.get(m_name)
    if motor is None:
        print("Motor '{}' chưa được setup_tc1508a()".format(m_name))
        return
    motor.brake()


def stop_all():
    """
    Dừng tất cả các motor đã setup.
    Dùng an toàn khi thoát chương trình.
    """
    for motor in _motors.values():
        motor.stop()
