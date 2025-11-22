# led_matrix8x8.py
#
# Wrapper cho LED ma trận 8x8 dùng HT16K33.
# Thiết kế để:
#   - Dùng trực tiếp với board 8x8 Dot Matrix Keyestudio (layout hơi khác Adafruit).
#   - Phục vụ sinh code từ Blockly (API đơn giản).
#
# Phụ thuộc:
#   - ht16k33.py
#   - ht16k33matrix.py  (Tony Smith, giữ nguyên)
#
# Đặc điểm:
#   - Không sửa thư viện gốc.
#   - Xử lý xoay (0/90/180/270) và dịch dọc (+2 hàng cho Keyestudio) ngay trong wrapper.


from machine import Pin, I2C

try:
    from ht16k33matrix import HT16K33Matrix
except ImportError:
    # Trường hợp đóng gói dạng package
    from .ht16k33matrix import HT16K33Matrix


# ================== CẤU HÌNH MẶC ĐỊNH ==================

# ESP32-S3 DevKitC-1 thường dùng I2C0 với SDA=8, SCL=9
_DEFAULT_I2C_ID = 0
_DEFAULT_SDA = 8
_DEFAULT_SCL = 9
_DEFAULT_ADDR = 0x70      # Adafruit backpack mặc định
_DEFAULT_FREQ = 400_000   # 400kHz

# Độ lệch dọc (bit) để match với ma trận Keyestudio
# +2 nghĩa là dịch "hình" lên trên 2 hàng
_DEFAULT_VERTICAL_OFFSET = 2


# ================== BIẾN TOÀN CỤC ==================

_matrix = None
_orientation_deg = 0
_vertical_offset = _DEFAULT_VERTICAL_OFFSET


# ================== HÀM NỘI BỘ ==================

def _ensure_init():
    """
    Đảm bảo ma trận đã khởi tạo.
    Nếu chưa, tự khởi tạo với cấu hình mặc định.
    """
    global _matrix

    if _matrix is not None:
        return

    i2c = I2C(
        _DEFAULT_I2C_ID,
        sda=Pin(_DEFAULT_SDA),
        scl=Pin(_DEFAULT_SCL),
        freq=_DEFAULT_FREQ,
    )
    m = HT16K33Matrix(i2c, i2c_address=_DEFAULT_ADDR)
    # KHÔNG dùng m.set_angle() của thư viện gốc
    m.set_brightness(8)
    m.clear()
    m.draw()
    _matrix = m


def _angle_to_code(deg):
    """
    Chuyển góc (0/90/180/270) sang code 0..3:
      0 -> 0
      90 -> 1
      180 -> 2
      270 -> 3
    """
    deg = int(deg) % 360
    if deg < 45 or deg >= 315:
        return 0
    if deg < 135:
        return 1
    if deg < 225:
        return 2
    return 3


def _rotate_buffer(buf, width, height, angle_code):
    """
    Xoay buffer dạng CỘT:
        buf[x] = byte, bit y = hàng y (0 = bottom, 7 = top).

    angle_code:
        0 = 0°
        1 = 90° CW
        2 = 180°
        3 = 270° (90° CCW)
    """
    if angle_code == 0:
        return bytearray(buf)

    out = bytearray(width)

    for x in range(width):
        col = buf[x]
        for y in range(height):
            if (col >> y) & 1:
                if angle_code == 1:      # 90° CW
                    nx = (width - 1) - y
                    ny = x
                elif angle_code == 2:    # 180°
                    nx = (width - 1) - x
                    ny = (height - 1) - y
                else:                    # 270° (90° CCW)
                    nx = y
                    ny = (height - 1) - x
                out[nx] |= (1 << ny)

    return out


def _shift_vertical(buf, offset):
    """
    Dịch hình theo trục dọc:
        offset > 0  -> dịch lên trên (về phía bit cao hơn)
        offset < 0  -> dịch xuống dưới
    """
    if offset == 0:
        return bytearray(buf)

    out = bytearray(len(buf))
    for x in range(len(buf)):
        col = buf[x]
        if offset > 0:
            out[x] = (col << offset) & 0xFF
        else:
            out[x] = (col >> (-offset)) & 0xFF
    return out


def _draw_with_transform():
    """
    Vẽ buffer với:
        - Xoay theo _orientation_deg
        - Dịch dọc theo _vertical_offset
    Không đụng đến is_rotated / set_angle() của thư viện gốc.
    """
    global _matrix, _orientation_deg, _vertical_offset

    _ensure_init()

    angle_code = _angle_to_code(_orientation_deg)

    width = _matrix.width
    height = _matrix.height

    # 1) copy buffer gốc
    original = bytearray(_matrix.buffer)

    # 2) xoay
    rotated = _rotate_buffer(original, width, height, angle_code)

    # 3) dịch dọc để khớp ma trận Keyestudio
    shifted = _shift_vertical(rotated, _vertical_offset)

    # 4) gán vào buffer và vẽ
    for i in range(width):
        _matrix.buffer[i] = shifted[i]
    _matrix.draw()

    # 5) trả lại buffer gốc để state nội bộ không bị ảnh hưởng
    for i in range(width):
        _matrix.buffer[i] = original[i]


# ================== HÀM PUBLIC CHO BLOCKLY ==================

def matrix_init(
    sda=_DEFAULT_SDA,
    scl=_DEFAULT_SCL,
    address=_DEFAULT_ADDR,
    brightness=8,
    i2c_id=_DEFAULT_I2C_ID,
    freq=_DEFAULT_FREQ,
    orientation=0,
    v_offset=_DEFAULT_VERTICAL_OFFSET,
):
    """
    Khởi tạo LED ma trận 8x8.

    Args:
        sda (int): GPIO SDA
        scl (int): GPIO SCL
        address (int): I2C address của HT16K33 (mặc định 0x70 -> 112)
        brightness (int): Độ sáng 0–15
        i2c_id (int): ID bus I2C (thường là 0)
        freq (int): Tần số I2C (Hz)
        orientation (int): Góc xoay 0/90/180/270 (theo chiều nhìn)
        v_offset (int): Độ dịch dọc (bit), Keyestudio thường cần 2
    """
    global _matrix, _orientation_deg, _vertical_offset

    _orientation_deg = int(orientation)
    _vertical_offset = int(v_offset)

    i2c = I2C(i2c_id, sda=Pin(sda), scl=Pin(scl), freq=freq)
    m = HT16K33Matrix(i2c, i2c_address=address)
    # Không dùng set_angle() của thư viện gốc
    lvl = int(brightness)
    if lvl < 0:
        lvl = 0
    if lvl > 15:
        lvl = 15
    m.set_brightness(lvl)
    m.clear()
    m.draw()
    _matrix = m


def matrix_set_orientation(angle):
    """
    Đặt góc xoay hiển thị (0 / 90 / 180 / 270).
    Dùng cho block kiểu: 'Xoay ma trận ___ độ'.
    """
    global _orientation_deg
    _orientation_deg = int(angle)


def matrix_set_vertical_offset(offset):
    """
    Đặt lại độ dịch dọc (bit).
    Dùng khi bạn dùng loại 8x8 khác layout Keyestudio.
    """
    global _vertical_offset
    _vertical_offset = int(offset)


def matrix_clear():
    """
    Xoá toàn bộ LED trên ma trận.
    """
    _ensure_init()
    _matrix.clear()
    _draw_with_transform()


def matrix_set_brightness(level):
    """
    Đặt độ sáng (0–15).
    """
    _ensure_init()
    lvl = int(level)
    if lvl < 0:
        lvl = 0
    if lvl > 15:
        lvl = 15
    _matrix.set_brightness(lvl)
    _draw_with_transform()


def matrix_show_char(ch, centre=True):
    """
    Hiển thị một ký tự đơn.

    Args:
        ch (str|int): ký tự (vd: 'A') hoặc mã ASCII (vd: 65)
        centre (bool): căn giữa ký tự
    """
    _ensure_init()

    if isinstance(ch, str):
        if not ch:
            return
        code = ord(ch[0])
    else:
        code = int(ch) & 0xFF

    _matrix.set_character(code, centre=centre)
    _draw_with_transform()


def matrix_show_text(text, speed=0.1):
    """
    Chạy chữ (scroll).

    Lưu ý:
        Hàm scroll_text của thư viện gốc tự xử lý draw() theo layout gốc,
        nên ở bản đơn giản này chữ chạy KHÔNG áp dụng orientation + offset.
        Nếu cần scroll cũng xoay + dịch, ta sẽ viết lại routine scroll riêng.
    """
    _ensure_init()
    _matrix.scroll_text(str(text), speed=float(speed))


def matrix_set_pixel(x, y, state=1, xor=False):
    """
    Bật/tắt 1 điểm tại (x, y).

    Args:
        x (int): 0–7
        y (int): 0–7 (0 là hàng dưới cùng theo thư viện gốc)
        state (bool|int): True/1 = bật, False/0 = tắt
        xor (bool): nếu True, vẽ XOR với trạng thái hiện tại
    """
    _ensure_init()
    xx = int(x)
    yy = int(y)
    ink = 1 if state else 0
    _matrix.plot(xx, yy, ink=ink, xor=xor)
    _draw_with_transform()


def matrix_fill(on=1):
    """
    Bật/tắt toàn bộ ma trận.

    Args:
        on (bool|int): True/1 = bật hết, False/0 = tắt hết
    """
    _ensure_init()
    if on:
        _matrix._fill(0xFF)
    else:
        _matrix.clear()
    _draw_with_transform()
