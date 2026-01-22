from machine import I2C, Pin
from time import sleep
from struct import pack
from micropython import const

# Địa chỉ I2C mặc định của module driver 4 motor
MD4C_DEFAULT_I2C_ADDRESS = 0x30

# Motor Index
MD4C_REG_CH1 = const(0)
MD4C_REG_CH2 = const(1)
MD4C_REG_CH3 = const(2)
MD4C_REG_CH4 = const(3)

# Direction
DIR_FORWARD = const(0)
DIR_BACKWARD = const(1)

# Stepper Style Controls
STEPPER_STYLE_SINGLE = const(0)
STEPPER_STYLE_DOUBLE = const(1)
STEPPER_STYLE_INTERLEAVE = const(2)

# Stepper Mode
STEPPER_MODE_SPEED = const(0)
STEPPER_MODE_STEP = const(1)

# Max Speed
DC_MOTOR_MAX_SPEED = 100
STEPPER_MOTOR_MAX_SPEED = 255

# ESP32-S3 DevKitC-1 I2C preset
S3_I2C0_SDA = 8
S3_I2C0_SCL = 9
S3_I2C1_SDA = 10
S3_I2C1_SCL = 11


class _MD4CBase:
    """
    Base class dùng chung HARD I2C cho driver MD4C.
    """

    def __init__(
        self,
        address=MD4C_DEFAULT_I2C_ADDRESS,
        scl_pin=None,
        sda_pin=None,
        freq=100_000,
        i2c=None,
        i2c_id=0,
    ):
        """
        address : địa chỉ I2C của driver
        scl_pin : số GPIO SCL (tuỳ chọn, nếu None -> auto)
        sda_pin : số GPIO SDA (tuỳ chọn, nếu None -> auto)
        freq    : tần số I2C (Hz)
        i2c     : có thể truyền sẵn 1 đối tượng I2C/SoftI2C để dùng chung
        i2c_id  : ID bus I2C (0 hoặc 1)
        """
        if i2c is not None:
            self._i2c = i2c
        else:
            # auto chọn chân nếu không truyền
            if scl_pin is None or sda_pin is None:
                if i2c_id == 0:
                    sda_pin = S3_I2C0_SDA
                    scl_pin = S3_I2C0_SCL
                elif i2c_id == 1:
                    sda_pin = S3_I2C1_SDA
                    scl_pin = S3_I2C1_SCL
                else:
                    sda_pin = S3_I2C0_SDA
                    scl_pin = S3_I2C0_SCL

            self._i2c = I2C(
                i2c_id,
                scl=Pin(scl_pin),
                sda=Pin(sda_pin),
                freq=freq,
            )

        self._addr = address

    def _write(self, register, data):
        """
        Ghi 1 gói 8 byte vào register (motor_type) của MD4C.
        """
        self._i2c.writeto_mem(self._addr, register, data)


class DCMotorV1(_MD4CBase):
    """
    Điều khiển DC Motor qua MD4C (4 kênh).
    """

    def __init__(self, address=MD4C_DEFAULT_I2C_ADDRESS,
                 scl_pin=None, sda_pin=None, freq=100_000, i2c=None, i2c_id=0):
        super().__init__(
            address=address,
            scl_pin=scl_pin,
            sda_pin=sda_pin,
            freq=freq,
            i2c=i2c,
            i2c_id=i2c_id,
        )
        self._motor_type = 0x00  # type 0x00 cho DC motor

        who_am_i = MD4C_DEFAULT_I2C_ADDRESS
        if who_am_i != MD4C_DEFAULT_I2C_ADDRESS:
            print(who_am_i)
            raise RuntimeError(
                "Could not find motor driver at address 0x{:X}".format(address)
            )
        else:
            # Tắt hết motor lúc khởi tạo
            self.setSpeed(MD4C_REG_CH1, 0)
            self.setSpeed(MD4C_REG_CH2, 0)
            self.setSpeed(MD4C_REG_CH3, 0)
            self.setSpeed(MD4C_REG_CH4, 0)
            print("DC Motor 4 channel driver initialized")

    # --- phần còn lại của DCMotorV1 giữ nguyên ---


    def setSpeed(self, motor_index, speed):
        """
        motor_index : 0..3
        speed       : -100..100 (âm là BACKWARD, dương là FORWARD)
        """
        if motor_index not in (MD4C_REG_CH1, MD4C_REG_CH2, MD4C_REG_CH3, MD4C_REG_CH4):
            raise RuntimeError("Invalid motor number")

        if speed < -DC_MOTOR_MAX_SPEED:
            speed = -DC_MOTOR_MAX_SPEED
        elif speed > DC_MOTOR_MAX_SPEED:
            speed = DC_MOTOR_MAX_SPEED

        if speed < 0:
            direction = DIR_BACKWARD
            speed = -speed
        else:
            direction = DIR_FORWARD

        # Gửi gói 8 byte
        data = pack(
            "BBBBBBBB",
            motor_index,
            direction,
            speed >> 8,
            speed & 0xFF,
            0,
            0,
            0,
            0,
        )
        self._write(self._motor_type, data)

    def set_motor_time(self, motor_index, speed, t=None):
        """
        Chạy motor trong t (giây) rồi tự dừng (setSpeed = 0).
        """
        if motor_index not in (MD4C_REG_CH1, MD4C_REG_CH2, MD4C_REG_CH3, MD4C_REG_CH4):
            raise RuntimeError("Invalid motor number")

        if speed < -DC_MOTOR_MAX_SPEED:
            speed = -DC_MOTOR_MAX_SPEED
        elif speed > DC_MOTOR_MAX_SPEED:
            speed = DC_MOTOR_MAX_SPEED

        if speed < 0:
            direction = DIR_BACKWARD
            speed = -speed
        else:
            direction = DIR_FORWARD

        data = pack(
            "BBBBBBBB",
            motor_index,
            direction,
            speed >> 8,
            speed & 0xFF,
            0,
            0,
            0,
            0,
        )
        self._write(self._motor_type, data)

        if t is not None:
            sleep(t)
            self.setSpeed(motor_index, 0)

    def fullOn(self, motor_index, direction=DIR_FORWARD):
        """
        Bật full speed theo chiều direction.
        direction: DIR_FORWARD hoặc DIR_BACKWARD
        """
        if direction == DIR_FORWARD:
            self.setSpeed(motor_index, DC_MOTOR_MAX_SPEED)
        else:
            self.setSpeed(motor_index, -DC_MOTOR_MAX_SPEED)

    def fullOff(self, motor_index):
        """
        Tắt motor (speed = 0).
        """
        self.setSpeed(motor_index, 0)


class StepperMotor(_MD4CBase):
    """
    Điều khiển Stepper qua MD4C (2 kênh).
    """

    def __init__(
        self,
        address=MD4C_DEFAULT_I2C_ADDRESS,
        number_step=200,
        scl_pin=None,
        sda_pin=None,
        freq=100_000,
        i2c=None,
        i2c_id=0,
    ):
        super().__init__(
            address=address,
            scl_pin=scl_pin,
            sda_pin=sda_pin,
            freq=freq,
            i2c=i2c,
            i2c_id=i2c_id,
        )
        self._motor_type = 0x01  # type 0x01 cho stepper
        self._number_step = number_step

        who_am_i = MD4C_DEFAULT_I2C_ADDRESS
        if who_am_i != MD4C_DEFAULT_I2C_ADDRESS:
            print(who_am_i)
            raise RuntimeError(
                "Could not find motor driver at address 0x{:X}".format(address)
            )
        else:
            self.setSpeed(MD4C_REG_CH1, DIR_FORWARD, 0)
            self.setSpeed(MD4C_REG_CH2, DIR_FORWARD, 0)
            print("Stepper Motor 2 channel driver initialized")

    # các hàm setSpeed, step, onestep, release giữ nguyên


    def setSpeed(self, motor_index, direction, speed, style=STEPPER_STYLE_INTERLEAVE):
        """
        Thiết lập chế độ chạy tốc độ (SPEED MODE)
        speed: 0..255
        """
        if motor_index not in (MD4C_REG_CH1, MD4C_REG_CH2):
            raise RuntimeError("Invalid motor number")

        if (speed < 0) or (speed > STEPPER_MOTOR_MAX_SPEED):
            raise RuntimeError("Speed is out of range")

        if direction not in (DIR_FORWARD, DIR_BACKWARD):
            raise RuntimeError("Direction is not valid")

        data = pack(
            "BBBBBBBB",
            motor_index,
            self._number_step >> 8,
            (self._number_step) & 0xFF,
            style,
            STEPPER_MODE_SPEED,
            direction,
            speed >> 8,
            speed & 0xFF,
        )
        self._write(self._motor_type, data)

    def step(self, motor_index, direction, steps, style=STEPPER_STYLE_DOUBLE):
        """
        Chạy stepper theo số bước (STEP MODE)
        steps: số bước cần quay
        """
        if motor_index not in (MD4C_REG_CH1, MD4C_REG_CH2):
            raise RuntimeError("Invalid motor number")

        if direction not in (DIR_FORWARD, DIR_BACKWARD):
            raise RuntimeError("Direction is not valid")

        data = pack(
            "BBBBBBBB",
            motor_index,
            self._number_step >> 8,
            (self._number_step) & 0xFF,
            style,
            STEPPER_MODE_STEP,
            direction,
            steps >> 8,
            steps & 0xFF,
        )
        self._write(self._motor_type, data)

    def onestep(self, motor_index, direction, style=STEPPER_STYLE_DOUBLE):
        """
        Quay 1 bước.
        """
        self.step(motor_index, direction, 1, style)

    def release(self, motor_index):
        """
        Nhả motor (set speed = 0).
        """
        self.setSpeed(motor_index, DIR_FORWARD, 0)
