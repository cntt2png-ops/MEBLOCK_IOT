# oled.py – Especially for ESP32-S3 DevKitC-1 (preset, HARD I2C only)

from machine import Pin, I2C
import framebuf

# ===== PRESET S3 DEVKIT C1 =====
# Định nghĩa 2 bus cứng:
#   - I2C0: SDA=8,  SCL=9
#   - I2C1: SDA=10, SCL=11
S3_I2C0_SDA = 8
S3_I2C0_SCL = 9
S3_I2C1_SDA = 10
S3_I2C1_SCL = 11

# Giữ alias cũ cho tương thích (mặc định bus 0)
S3_I2C_SDA = S3_I2C0_SDA
S3_I2C_SCL = S3_I2C0_SCL

# -------------------- Drivers --------------------

# ---- SSD1306 ----
# mạch điều khiển có bộ nhớ bằng đúng bề ngang (128), show() gửi đúng 128/cột mỗi page
class _SSD1306:
    def __init__(self, width, height, external_vcc=False):
        self.width, self.height = width, height
        self.external_vcc = external_vcc
        self.pages = self.height // 8
        self.buffer = bytearray(self.width * self.pages)
        self.fb = framebuf.FrameBuffer(self.buffer, self.width, self.height, framebuf.MONO_VLSB)
        self._init()

    def _cmd(self, *cmds): raise NotImplementedError
    def _data(self, buf):  raise NotImplementedError

    def _init(self):
        # các hằng số lệnh
        SET_CONTRAST=0x81; DISPLAY_ALL_ON_RES=0xA4; NORMAL_DISPLAY=0xA6
        DISPLAY_OFF=0xAE; DISPLAY_ON=0xAF; SET_DISPLAY_OFFSET=0xD3
        SET_COMPINS=0xDA; SET_VCOM_DETECT=0xDB; SET_DISPLAY_CLOCK=0xD5
        SET_PRECHARGE=0xD9; SET_MULTIPLEX=0xA8; MEMORY_MODE=0x20
        COM_SCAN_DEC=0xC8; SEG_REMAP=0xA0; CHARGE_PUMP=0x8D
        COLUMN_ADDR=0x21
        self._cmd(DISPLAY_OFF)
        self._cmd(SET_DISPLAY_CLOCK, 0x80)
        self._cmd(SET_MULTIPLEX, self.height - 1)
        self._cmd(SET_DISPLAY_OFFSET, 0x00)
        self._cmd(0x40)  # start line = 0
        self._cmd(CHARGE_PUMP, 0x10 if self.external_vcc else 0x14)
        self._cmd(MEMORY_MODE, 0x00)     # horizontal
        self._cmd(SEG_REMAP | 0x01)
        self._cmd(COM_SCAN_DEC)
        self._cmd(SET_COMPINS, 0x02 if self.height == 32 else 0x12)
        self._cmd(SET_CONTRAST, 0x8F if self.external_vcc else 0xCF)
        self._cmd(SET_PRECHARGE, 0x22 if self.external_vcc else 0xF1)
        self._cmd(SET_VCOM_DETECT, 0x40)
        self._cmd(DISPLAY_ALL_ON_RES, NORMAL_DISPLAY, DISPLAY_ON)
        self.fill(0); self.show()

    # tiện ích nguồn/độ tương phản/đảo màu
    def poweroff(self): self._cmd(0xAE)
    def poweron(self):  self._cmd(0xAF)
    def contrast(self, c): self._cmd(0x81, c & 0xFF)
    def invert(self, inv): self._cmd(0xA7 if inv else 0xA6)

    # framebuffer helpers
    def fill(self, c): self.fb.fill(c)
    def pixel(self, x, y, c): self.fb.pixel(x, y, c)
    def scroll(self, dx, dy): self.fb.scroll(dx, dy)
    def text(self, s, x, y, c=1): self.fb.text(s, x, y, c)
    def line(self, x1, y1, x2, y2, c=1): self.fb.line(x1, y1, x2, y2, c)
    def hline(self, x, y, w, c=1): self.fb.hline(x, y, w, c)
    def vline(self, x, y, h, c=1): self.fb.vline(x, y, h, c)
    def rect(self, x, y, w, h, c=1): self.fb.rect(x, y, w, h, c)
    def fill_rect(self, x, y, w, h, c=1): self.fb.fill_rect(x, y, w, h, c)

    def show(self):
        COLUMN_ADDR=0x21
        for page in range(self.pages):
            # set page (0xB0..)
            self._cmd(0xB0 | page, 0x00, 0x10, COLUMN_ADDR, 0, self.width - 1)
            start = page * self.width
            self._data(self.buffer[start:start + self.width])


class SSD1306_I2C(_SSD1306):
    def __init__(self, width, height, i2c, addr=0x3C, external_vcc=False):
        self.i2c, self.addr = i2c, addr
        super().__init__(width, height, external_vcc)
    def _cmd(self, *cmds):
        for c in cmds:
            self.i2c.writeto(self.addr, b'\x80' + bytes((c,)))
    def _data(self, buf):
        self.i2c.writeto(self.addr, b'\x40' + buf)

# ---- SH1106 ----
# Bộ nhớ 132 cột → show() luôn ghi đủ 132 byte mỗi page.
# Dùng x_offset (thường 2) để canh vùng hiển thị 128 cột trong 132.
class SH1106_I2C:
    SH_WIDTH = 132
    def __init__(self, width, height, i2c, addr=0x3C, x_offset=2):
        self.width, self.height = width, height
        self.i2c, self.addr = i2c, addr
        self.x_offset = x_offset
        self.pages = self.height // 8
        self.buffer = bytearray(self.width * self.pages)
        self.fb = framebuf.FrameBuffer(self.buffer, self.width, self.height, framebuf.MONO_VLSB)
        self._line = bytearray(self.SH_WIDTH)  # buffer 132 byte cho mỗi page
        self._init()

    def _cmd(self, c):  self.i2c.writeto(self.addr, b"\x00" + bytes([c]))
    def _data(self, b): self.i2c.writeto(self.addr, b"\x40" + b)

    def _init(self):
        # dàn lệnh phổ biến
        for c in (0xAE,       # OFF
                  0xA1,       # segment remap
                  0xC8,       # COM scan dir remap
                  0xA8, self.height-1,
                  0xD3, 0x00, # display offset
                  0xD5, 0x80, # clk
                  0xD9, 0x22, # precharge
                  0xDB, 0x35, # vcom
                  0x81, 0xCF, # contrast
                  0xA6,       # normal
                  0xA4,       # display from RAM
                  0xAF):      # ON
            self._cmd(c)
        self.fill(0); self.show()

    def poweroff(self): self._cmd(0xAE)
    def poweron(self):  self._cmd(0xAF)
    def contrast(self, c): self._cmd(0x81); self._cmd(c & 0xFF)
    def invert(self, inv): self._cmd(0xA7 if inv else 0xA6)

    # framebuffer helpers
    def fill(self, c): self.fb.fill(c)
    def pixel(self, x, y, c): self.fb.pixel(x, y, c)
    def scroll(self, dx, dy): self.fb.scroll(dx, dy)
    def text(self, s, x, y, c=1): self.fb.text(s, x, y, c)
    def line(self, x1, y1, x2, y2, c=1): self.fb.line(x1, y1, x2, y2, c)
    def hline(self, x, y, w, c=1): self.fb.hline(x, y, w, c)
    def vline(self, x, y, h, c=1): self.fb.vline(x, y, h, c)
    def rect(self, x, y, w, h, c=1): self.fb.rect(x, y, w, h, c)
    def fill_rect(self, x, y, w, h, c=1): self.fb.fill_rect(x, y, w, h, c)

    def show(self):
        w = self.width
        left_pad  = max(0, min(self.x_offset, self.SH_WIDTH))
        right_pad = max(0, self.SH_WIDTH - left_pad - w)
        for page in range(self.pages):
            # chuẩn bị line 132: [0..0][128 data][0..0]
            if left_pad:
                for i in range(left_pad): self._line[i] = 0x00
            start = page * w
            self._line[left_pad:left_pad+w] = self.buffer[start:start+w]
            if right_pad:
                for i in range(left_pad+w, self.SH_WIDTH): self._line[i] = 0x00
            # chọn page + cột = 0
            self._cmd(0xB0 + page)
            self._cmd(0x10)   # high nibble
            self._cmd(0x00)   # low nibble
            self._data(self._line)


# ===== HARD I2C ONLY, 2 BUS (8/9 và 10/11) =====
def _make_i2c(i2c, i2c_id, sda, scl, freq):
    """
    Luôn dùng hardware I2C.
    - Nếu i2c != None → dùng lại object đó.
    - Nếu không:
        + Nếu sda/scl None → chọn theo i2c_id:
            i2c_id = 0 → SDA=8,  SCL=9
            i2c_id = 1 → SDA=10, SCL=11
        + Nếu sda/scl được truyền → dùng đúng như vậy.
    """
    if i2c:
        return i2c

    # Auto chọn chân theo bus nếu chưa được truyền
    if sda is None or scl is None:
        if i2c_id == 0:
            sda = S3_I2C0_SDA
            scl = S3_I2C0_SCL
        elif i2c_id == 1:
            sda = S3_I2C1_SDA
            scl = S3_I2C1_SCL
        else:
            # fallback: bus 0
            sda = S3_I2C0_SDA
            scl = S3_I2C0_SCL

    return I2C(i2c_id, scl=Pin(scl), sda=Pin(sda), freq=freq)


def _auto_addr(i2c, addr):
    if addr is not None:
        return addr
    devs = i2c.scan()
    if not devs:
        raise RuntimeError("Cannot find I2C device.")
    return 0x3C if 0x3C in devs else (0x3D if 0x3D in devs else devs[0])


def create(*,
           i2c=None, i2c_id=0, sda=None, scl=None, freq=400_000,
           width=128, height=64,
           ctrl="SSD1306", sh1106_col_offset=2,
           addr=None, backend="hard",
           debug=False):
    """
    Tạo đối tượng OLED:
    - Luôn dùng HARD I2C, backend giữ lại cho tương thích nhưng bị bỏ qua.
    - i2c_id = 0 → SDA=8, SCL=9 (nếu không truyền sda/scl)
    - i2c_id = 1 → SDA=10, SCL=11 (nếu không truyền sda/scl)
    """
    _sda = sda
    _scl = scl

    i2c_obj = _make_i2c(i2c, i2c_id, _sda, _scl, freq)
    a = _auto_addr(i2c_obj, addr)
    ctrl_up = (ctrl or "SSD1306").upper()
    if ctrl_up == "SSD1306":
        dev = SSD1306_I2C(width, height, i2c_obj, addr=a)
    elif ctrl_up == "SH1106":
        dev = SH1106_I2C(width, height, i2c_obj, addr=a, x_offset=sh1106_col_offset)
    else:
        raise ValueError('ctrl must "SSD1306" or "SH1106"')

    def _clear():
        dev.fill(0)
        dev.show()
    dev.clear = _clear
    return dev


class Oled:
    def __init__(self, width=128, height=64, *,
                 sda=None, scl=None,
                 driver="SSD1306",
                 addr=None,
                 backend="hard",  
                 i2c_id=0, freq=400_000,
                 sh1106_offset=2,
                 debug=False):
        """
        Oled high-level wrapper.

        Ví dụ:
            # Bus 0: SDA=8, SCL=9
            oled0 = Oled()

            # Bus 1: SDA=10, SCL=11
            oled1 = Oled(i2c_id=1)

            # Dùng chân custom nhưng vẫn là HARD I2C:
            oled_custom = Oled(sda=4, scl=5)
        """
        self.width, self.height = width, height
        self.dev = create(
            i2c=None, i2c_id=i2c_id,
            sda=sda, scl=scl, freq=freq,
            width=width, height=height,
            ctrl=driver, sh1106_col_offset=sh1106_offset,
            addr=addr, backend="hard", debug=debug
        )

    def clear(self): self.dev.clear()
    def fill(self, c=1): self.dev.fill(c)
    def show(self): self.dev.show()
    def text(self, s, x, y, c=1): self.dev.text(str(s), x, y, c)

    def text_wrap(self, s, x=0, y=0, c=1, line_h=8, char_w=8):
        max_cols = max(1, self.width // char_w)
        cy = y; line = ""
        for ch in str(s):
            if ch == "\n" or len(line) >= max_cols:
                self.dev.text(line, x, cy, c); cy += line_h
                line = "" if ch == "\n" else ch
                if cy > (self.height - line_h): break
            else:
                line += ch
        if line and cy <= (self.height - line_h):
            self.dev.text(line, x, cy, c)

    def poweroff(self):
        if hasattr(self.dev, "poweroff"): self.dev.poweroff()

    def poweron(self):
        if hasattr(self.dev, "poweron"): self.dev.poweron()

    def invert(self, inv):
        if hasattr(self.dev, "invert"): self.dev.invert(bool(inv))

    def contrast(self, val):
        if hasattr(self.dev, "contrast"): self.dev.contrast(int(val) & 0xFF)

    def __getattr__(self, name):
        return getattr(self.dev, name)
