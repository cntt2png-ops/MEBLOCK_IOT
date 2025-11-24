from machine import SoftI2C, Pin
from i2c_motors_driver import DCMotorV1, MD4C_REG_CH1, MD4C_REG_CH2, MD4C_REG_CH3, MD4C_REG_CH4
from esp32_gamepad import GamepadReceiver
import time

# ==============================
# CẤU HÌNH
# ==============================

# I2C điều khiển module 4 motor MD4C

# Tốc độ tối đa (0..100)
MAX_SPEED = 80

# ==============================
# KHỞI TẠO PHẦN CỨNG
# ==============================

# Driver 4 DC motor
dc = DCMotorV1(scl_pin=10, sda_pin=11)

# Gamepad receiver
gp = GamepadReceiver(scl_pin=10, sda_pin=11, verbose=False)

# ==============================
# HÀM PHỤ TRỢ MOTOR
# ==============================
def set_left_speed(speed):
    """Set tốc độ cho 2 motor bên trái (CH1, CH2). speed: -100..100"""
    dc.setSpeed(MD4C_REG_CH1, speed)
    dc.setSpeed(MD4C_REG_CH2, speed)

def set_right_speed(speed):
    """Set tốc độ cho 2 motor bên phải (CH3, CH4). speed: -100..100"""
    dc.setSpeed(MD4C_REG_CH3, speed)
    dc.setSpeed(MD4C_REG_CH4, speed)

def stop():
    """Dừng toàn bộ motor."""
    set_left_speed(0)
    set_right_speed(0)

# ==============================
# ĐIỀU KHIỂN D-PAD
# ==============================

def drive_dpad(d):
    """
    Điều khiển bằng D-pad.

    d: dict dữ liệu gamepad (gp.data)
    Trả về:
        True  nếu đã dùng D-pad để điều khiển
        False nếu không bấm D-pad (để fallback sang joystick)
    """
    up = d['dpad_up']
    down = d['dpad_down']
    left = d['dpad_left']
    right = d['dpad_right']

    if not (up or down or left or right):
        return False
    
    if up and not down:
        # Chạy thẳng tới
        print("up")
        set_left_speed(100)

    elif down and not up:
        print("dowwn")
        # Chạy lùi
        set_left_speed(-100)

    elif left and not right:
        print("lefft")
        # Quay tại chỗ sang trái (trái lùi, phải tiến)
        set_right_speed(100)

    elif right and not left:
        print("right")
        # Quay tại chỗ sang phải (trái tiến, phải lùi)
        set_right_speed(-100)

    else:
        # tạm dừng
        stop()

    return True

# ==============================
# ĐIỀU KHIỂN JOYSTICK TRÁI
# ==============================

def clamp(v, mi, ma):
    if v < mi:
        return mi
    if v > ma:
        return ma
    return v

def drive_joystick():
    """
    Dùng joystick trái (index=0) để điều khiển vi sai:
        left  = y + x
        right = y - x
    """
    x, y, angle, direction, dist = gp.read_joystick(0)

    # Nếu joystick gần giữa -> dừng
    if dist == 0:
        stop()
        return

    left = y + x
    right = y - x

    # Giới hạn -100..100
    left = clamp(left, -100, 100)
    right = clamp(right, -100, 100)

    # Scale theo MAX_SPEED
    scale = MAX_SPEED / 100.0
    left *= scale
    right *= scale

    dc.setSpeed(MD4C_REG_CH2, int(left))
    dc.setSpeed(MD4C_REG_CH4, int(right))

# ==============================
# VÒNG LẶP CHÍNH
# ==============================

print(">> Gamepad + 4 motor READY")

try:
    while True:
        gp.update()

        # Gamepad chưa kết nối -> dừng xe
        if not gp.is_connected():
            stop()
            time.sleep_ms(100)
            continue
        d = gp.data

        # ƯU TIÊN D-PAD
        used_dpad = drive_dpad(d)

        # Nếu không bấm D-pad -> lái bằng joystick trái
        if not used_dpad:
            drive_joystick()

        time.sleep_ms(20)

except KeyboardInterrupt:
    stop()
    print("Stopped.")
