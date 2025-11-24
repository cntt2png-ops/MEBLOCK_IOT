from esp32_gamepad import GamepadReceiver
import time
from i2c_motors_driver import (
    DCMotorV1,
    MD4C_REG_CH1,
    MD4C_REG_CH2,
    MD4C_REG_CH3,
    MD4C_REG_CH4,
)
# Khởi tạo I2C và receiver
gp = GamepadReceiver(scl_pin=10, sda_pin=11, verbose=False)

# Lưu trạng thái cũ để chỉ in khi có thay đổi
last_data = None

def print_header():
    print("=" * 60)
    print("   TEST GAMEPAD RECEIVER - NHẤN TỪNG NÚT ĐỂ XEM TRẠNG THÁI")
    print("   D-PAD: up/down/left/right")
    print("   BUTTONS: A, B, X, Y, L1, R1, L2, R2, THUMBL, THUMBR, M1, M2, SYS")
    print("   JOYSTICKS: Left/Right (X,Y), Trigger L2/R2 analog (al2, ar2)")
    print("=" * 60)

def format_onoff(v):
    return "1" if v else "0"

print_header()

while True:
    gp.update()

    if not gp.is_connected():
        print(">>> Chưa kết nối gamepad hoặc receiver không có dữ liệu...")
        time.sleep(1)
        continue

    d = gp.data

    # Chỉ in khi dữ liệu thay đổi so với lần trước
    if last_data is None or d != last_data:
        print("\n----------------- TRẠNG THÁI HIỆN TẠI -----------------")

        # D-PAD
        print("DPAD raw:", d['dpad'],
              "| UP:", format_onoff(d['dpad_up']),
              "DOWN:", format_onoff(d['dpad_down']),
              "LEFT:", format_onoff(d['dpad_left']),
              "RIGHT:", format_onoff(d['dpad_right']))

        # JOYSTICK TRÁI & PHẢI (giá trị thô)
        print("Joystick LEFT  (alx, aly):", d['alx'], d['aly'])
        print("Joystick RIGHT (arx, ary):", d['arx'], d['ary'])

        # JOYSTICK chuẩn hóa (đọc bằng hàm tiện ích)
        lx, ly, langle, ldir, ldist = gp.read_joystick(0)
        rx, ry, rangle, rdir, rdist = gp.read_joystick(1)
        print("Joy LEFT norm : x=%3d y=%3d angle=%3d dir=%d dist=%3d"
              % (lx, ly, langle, ldir, ldist))
        print("Joy RIGHT norm: x=%3d y=%3d angle=%3d dir=%d dist=%3d"
              % (rx, ry, rangle, rdir, rdist))

        # TRIGGER ANALOG
        print("Trigger analog: AL2=%4d  AR2=%4d"
              % (d['al2'], d['ar2']))

        # CÁC NÚT DIGITAL
        print("Buttons: A:%s B:%s X:%s Y:%s  L1:%s R1:%s  L2:%s R2:%s"
              % (format_onoff(d['a']),
                 format_onoff(d['b']),
                 format_onoff(d['x']),
                 format_onoff(d['y']),
                 format_onoff(d['l1']),
                 format_onoff(d['r1']),
                 format_onoff(d['l2']),
                 format_onoff(d['r2'])))

        print("Thumb:  THUMBL:%s  THUMBR:%s"
              % (format_onoff(d.get('thumbl', 0)),
                 format_onoff(d.get('thumbr', 0))))

        print("Misc:   M1:%s  M2:%s  SYS:%s"
              % (format_onoff(d['m1']),
                 format_onoff(d['m2']),
                 format_onoff(d['sys'])))

        # Lưu lại để so sánh lần sau
        last_data = d.copy()

    time.sleep_ms(50)
