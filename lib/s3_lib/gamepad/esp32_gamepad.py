"""
Thư viện nhận dữ liệu từ Gamepad Receiver (I2C, addr 0x55) cho ESP32 (MicroPython).

- Hỗ trợ đọc:
    + D-pad (trên, dưới, trái, phải)
    + 2 joystick (trái/phải, trục X/Y)
    + Các nút A, B, X, Y, L1, R1, L2, R2, M1, M2, SYS
    + Giá trị analog của trigger L2/R2

- Cách dùng tối thiểu:

    from machine import Pin
    from esp32_gamepad import GamepadReceiver
    import time

    gp = GamepadReceiver(scl_pin=22, sda_pin=21)

    while True:
        gp.update()
        if gp.is_connected():
            print(gp.data)      # dict dữ liệu đầy đủ
            # Hoặc:
            # print(gp.read_joystick(0))  # joystick trái
        time.sleep_ms(50)

"""

from machine import I2C, Pin
from time import ticks_ms
import math

# Địa chỉ I2C của receiver
_GAMEPAD_RECEIVER_ADDR = 0x55

# ESP32-S3 DevKitC-1 I2C preset
S3_I2C0_SDA = 8
S3_I2C0_SCL = 9
S3_I2C1_SDA = 10
S3_I2C1_SCL = 11

# Các thanh ghi cấu hình (cho LED, rung)
_REG_SET_LED_COLOR = 0x01
_REG_SET_LED_PLAYER = 0x02
_REG_SET_RUMBLE = 0x03

# Bit trong dpad
_DPAD_UP = 0
_DPAD_DOWN = 1
_DPAD_RIGHT = 2
_DPAD_LEFT = 3

# Index bit trong field "buttons"
_BUTTON_A = 0
_BUTTON_B = 1
_BUTTON_X = 2
_BUTTON_Y = 3
_BUTTON_SHOULDER_L = 4
_BUTTON_SHOULDER_R = 5
_BUTTON_TRIGGER_L = 6
_BUTTON_TRIGGER_R = 7
_BUTTON_THUMB_L = 8
_BUTTON_THUMB_R = 9

# Index bit trong field "misc_buttons"
_MISC_BUTTON_SYSTEM = 0  # PS/Xbox/Home
_MISC_BUTTON_M1 = 1      # Select/Share/-
_MISC_BUTTON_M2 = 2      # Start/Options/+


def _read_16(b1, b2):
    """Đọc 16-bit signed từ 2 byte (little-endian)."""
    raw = (b1 << 8) | b2
    if raw & (1 << 15):
        return raw - (1 << 16)
    return raw


def _read_32(b1, b2, b3, b4):
    """Đọc 32-bit signed từ 4 byte (little-endian)."""
    raw = (b1 << 24) | (b2 << 16) | (b3 << 8) | b4
    if raw & (1 << 31):
        return raw - (1 << 32)
    return raw


def _translate(x, in_min, in_max, out_min, out_max):
    """Map giá trị từ khoảng [in_min, in_max] sang [out_min, out_max]."""
    # Tránh chia 0
    if in_max == in_min:
        return out_min
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


class GamepadReceiver:
    """
    Lớp giao tiếp với Gamepad Receiver qua I2C.
    """

    def __init__(self, scl_pin=None, sda_pin=None,
                 freq=100_000, i2c=None, verbose=False, i2c_id=0):
        """
        Khởi tạo receiver.

        :param scl_pin: GPIO SCL (nếu None -> auto)
        :param sda_pin: GPIO SDA (nếu None -> auto)
        :param freq: tần số I2C (Hz)
        :param i2c: có I2C sẵn thì truyền vào (I2C hoặc SoftI2C)
        :param verbose: True để in log debug
        :param i2c_id: 0 (SDA=8,SCL=9) hoặc 1 (SDA=10,SCL=11)
        """
        if i2c is None:
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

            self._i2c = I2C(i2c_id, scl=Pin(scl_pin), sda=Pin(sda_pin), freq=freq)
        else:
            # Dùng I2C / SoftI2C bên ngoài truyền vào
            self._i2c = i2c

        self._verbose = verbose
        self._last_print = ticks_ms()
        self._is_connected = False

        # Dữ liệu đã chuyển đổi
        self.data = {
            'dpad': 0,
            'dpad_left': 0,
            'dpad_right': 0,
            'dpad_up': 0,
            'dpad_down': 0,
            'alx': 0,
            'aly': 0,
            'arx': 0,
            'ary': 0,
            'a': 0,
            'b': 0,
            'x': 0,
            'y': 0,
            'l1': 0,
            'r1': 0,
            'l2': 0,
            'r2': 0,
            'al2': 0,
            'ar2': 0,
            'm1': 0,
            'm2': 0,
            'sys': 0,
        }

        # Raw values
        self.dpad = 0
        self.aLX = 0
        self.aLY = 0
        self.aRX = 0
        self.aRY = 0
        self.al2 = 0
        self.ar2 = 0
        self.buttons = 0
        self.misc_buttons = 0

    # =========================
    #   Public API
    # =========================

    def is_connected(self):
        """Trả về True nếu receiver báo có dữ liệu từ tay cầm."""
        return self._is_connected

    def update(self):
        """
        Đọc 30 byte từ receiver, parse, và cập nhật self.data.

        Gọi hàm này trong vòng lặp chính, ví dụ mỗi 10–50ms.
        """
        try:
            result = self._i2c.readfrom(_GAMEPAD_RECEIVER_ADDR, 30)
        except OSError:
            # Không đọc được (mất kết nối I2C, chưa cắm receiver, v.v.)
            self._is_connected = False
            self._reset_raw()
            self._convert_data()
            if self._verbose and ticks_ms() - self._last_print > 500:
                print("[GamepadReceiver] Không đọc được dữ liệu I2C")
                self._last_print = ticks_ms()
            return

        has_data = result[0]

        self._is_connected = (has_data == 1)

        if has_data:
            # Parse dữ liệu thô
            self.dpad = result[1]

            self.aLX = _read_32(result[2], result[3], result[4], result[5])
            self.aLY = _read_32(result[6], result[7], result[8], result[9])

            self.aRX = _read_32(result[10], result[11], result[12], result[13])
            self.aRY = _read_32(result[14], result[15], result[16], result[17])

            self.al2 = _read_32(result[18], result[19], result[20], result[21])
            self.ar2 = _read_32(result[22], result[23], result[24], result[25])

            self.buttons = _read_16(result[26], result[27])
            self.misc_buttons = _read_16(result[28], result[29])
        else:
            # Không có dữ liệu: reset về 0
            self._reset_raw()

        self._convert_data()

        if self._verbose and ticks_ms() - self._last_print > 100:
            print("dpad=", self.dpad,
                  " aLX=", self.aLX, " aLY=", self.aLY,
                  " aRX=", self.aRX, " aRY=", self.aRY,
                  " al2=", self.al2, " ar2=", self.ar2,
                  " buttons=", self.buttons, " misc=", self.misc_buttons)
            self._last_print = ticks_ms()

    def read_joystick(self, index=0):
        """
        Đọc joystick đã được chuẩn hóa.

        :param index: 0 = joystick trái, 1 = joystick phải
        :return: (x, y, angle, dir, distance)
            - x, y: -100 → 100
            - angle: 0–360 độ (hoặc -1 nếu không đủ mạnh)
            - dir: hướng 1..8
            - distance: 0–100 (mức độ gạt cần)
        """

        x = 0
        y = 0

        if index == 0:
            # Joystick trái: aLX, aLY
            if self.aLX < 0:
                x = round(_translate(self.aLX, -512, 0, 100, 0))
            else:
                x = round(_translate(self.aLX, 0, 512, 0, -100))

            if self.aLY < 0:
                y = round(_translate(self.aLY, -512, 0, 100, 0))
            else:
                y = round(_translate(self.aLY, 0, 512, 0, -100))
        else:
            # Joystick phải: aRX, aRY
            if self.aRX < 0:
                x = round(_translate(self.aRX, -512, 0, 100, 0))
            else:
                x = round(_translate(self.aRX, 0, 512, 0, -100))

            if self.aRY < 0:
                y = round(_translate(self.aRY, -512, 0, 100, 0))
            else:
                y = round(_translate(self.aRY, 0, 512, 0, -100))

        # Độ dài vector joystick (để làm speed)
        distance = int(math.sqrt(x * x + y * y))

        # Góc (theo độ) so với trục X dương
        angle = int((math.atan2(y, x) - math.atan2(0, 100)) * 180 / math.pi)
        if angle < 0:
            angle += 360

        # Nếu gạt quá nhẹ thì coi như 0
        if distance < 15:
            distance = 0
            angle = -1
        elif distance > 100:
            distance = 100

        dir_ = self._calculate_direction(angle) if angle >= 0 else 0

        return x, y, angle, dir_, distance

    def set_led_color(self, r, g, b):
        """
        Đổi màu LED trên receiver (nếu được hỗ trợ).

        :param r: 0-255
        :param g: 0-255
        :param b: 0-255
        """
        if not (0 <= r <= 255 and 0 <= g <= 255 and 0 <= b <= 255):
            return
        try:
            self._i2c.writeto_mem(_GAMEPAD_RECEIVER_ADDR,
                                  _REG_SET_LED_COLOR,
                                  bytes([r, g, b]))
        except OSError:
            pass

    def set_player_led(self, value):
        """
        Đổi LED player (1,2,3,4...) nếu receiver hỗ trợ.

        :param value: 0–255
        """
        if not (0 <= value <= 255):
            return
        try:
            self._i2c.writeto_mem(_GAMEPAD_RECEIVER_ADDR,
                                  _REG_SET_LED_PLAYER,
                                  bytes([value]))
        except OSError:
            pass

    def set_rumble(self, force, duration):
        """
        Kích hoạt rung tay cầm.

        :param force: 0–255
        :param duration: 0–255 (ms/đơn vị tuỳ firmware receiver)
        """
        if not (0 <= force <= 255 and 0 <= duration <= 255):
            return
        try:
            self._i2c.writeto_mem(_GAMEPAD_RECEIVER_ADDR,
                                  _REG_SET_RUMBLE,
                                  bytes([force, duration]))
        except OSError:
            pass

    # =========================
    #   Internal helpers
    # =========================

    def _reset_raw(self):
        """Reset các giá trị raw về 0."""
        self.dpad = 0
        self.aLX = 0
        self.aLY = 0
        self.aRX = 0
        self.aRY = 0
        self.al2 = 0
        self.ar2 = 0
        self.buttons = 0
        self.misc_buttons = 0

    def _convert_data(self):
        """Chuyển dữ liệu raw sang dict self.data tiện dùng."""
        self.data = {
            'dpad': self.dpad,
            'dpad_left': (self.dpad >> _DPAD_LEFT) & 1,
            'dpad_right': (self.dpad >> _DPAD_RIGHT) & 1,
            'dpad_up': (self.dpad >> _DPAD_UP) & 1,
            'dpad_down': (self.dpad >> _DPAD_DOWN) & 1,

            'alx': self.aLX,
            'aly': self.aLY,
            'arx': self.aRX,
            'ary': self.aRY,

            'thumbl': (self.buttons >> _BUTTON_THUMB_L) & 1,
            'thumbr': (self.buttons >> _BUTTON_THUMB_R) & 1,
            'a': (self.buttons >> _BUTTON_A) & 1,
            'b': (self.buttons >> _BUTTON_B) & 1,
            'x': (self.buttons >> _BUTTON_X) & 1,
            'y': (self.buttons >> _BUTTON_Y) & 1,
            'l1': (self.buttons >> _BUTTON_SHOULDER_L) & 1,
            'r1': (self.buttons >> _BUTTON_SHOULDER_R) & 1,
            'l2': (self.buttons >> _BUTTON_TRIGGER_L) & 1,
            'r2': (self.buttons >> _BUTTON_TRIGGER_R) & 1,

            'al2': self.al2,
            'ar2': self.ar2,

            'm1': (self.misc_buttons >> _MISC_BUTTON_M1) & 1,
            'm2': (self.misc_buttons >> _MISC_BUTTON_M2) & 1,
            'sys': (self.misc_buttons >> _MISC_BUTTON_SYSTEM) & 1,
        }

    def _calculate_direction(self, angle):
        """
        Tính hướng (1..8) từ góc:

                 90 (3)
            135(4) | 45(2)
        180 (5) ---+--- 0/360 (1)
            225(6) | 315(8)
                270 (7)
        """
        if angle < 0:
            return 0

        if 0 <= angle < 22.5 or angle >= 337.5:
            return 1
        elif 22.5 <= angle < 67.5:
            return 2
        elif 67.5 <= angle < 112.5:
            return 3
        elif 112.5 <= angle < 157.5:
            return 4
        elif 157.5 <= angle < 202.5:
            return 5
        elif 202.5 <= angle < 247.5:
            return 6
        elif 247.5 <= angle < 292.5:
            return 7
        elif 292.5 <= angle < 337.5:
            return 8

        return 0
