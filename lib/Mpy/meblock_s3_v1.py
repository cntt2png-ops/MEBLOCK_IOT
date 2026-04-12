# meblock_s3_v1.py
# MEBLOCK S3 V1.1 – "thư viện tổng" cho Blockly (lazy import + port map)
#
# Mục tiêu:
#   - Import 1 lần: import meblock_s3_v1 as mb
#   - Chỉ khi dùng vệ tinh mới import: ext_dht / oled / servo / ultrasonic
#   - Tham số truyền theo "tên cổng" (PORT) thay vì pin
#
# Yêu cầu đặt file:
#   - meblock_s3_v1.py
#   - ext_dht.py
#   - oled.py
#   - servo.py
#   - ultrasonic.py
# cùng thư mục trên thiết bị (hoặc trong sys.path).
#
# Ví dụ Blockly (gợi ý):
#   import meblock_s3_v1 as mb
#   mb.oled.text("Hello", 0, 0); mb.oled.show()
#   t = mb.dht.temperature()  # mặc định dht20 @ I2C0
#   d = mb.ultrasonic.distance_cm("DOUT1")
#   mb.servo.deg180("S_1", 90, 80)

try:
    from micropython import const
except Exception:
    def const(x): return x

# =========================
# 1) PORT MAP (V1.1)
# =========================
# Quy ước:
#   - PORT 2-pin: trả về (PIN1, PIN2)
#   - I2C: (SDA, SCL)
#   - UART: (TX, RX)
#   - Analog/Digital: (x1, x2)
PORT_MAP = {
    # I2C
    "I2C0": (8, 9),     # SDA, SCL
    "I2C1": (10, 11),   # SDA, SCL

    # Analog (Ax1, Ax2)
    "AIN1": (15, 16),
    "AIN2": (4, 5),
    "AIN3": (6, 7),
    "AIN4": (12, 13),

    # UART
    "UART": (17, 18),   # TX, RX

    # Digital (Dx1, Dx2)
    "DOUT1": (1, 2),
    "DOUT2": (39, 38),
    "DOUT3": (37, 36),
    "DOUT4": (21, 14),
}

# Nhãn in trên board -> PORT (tuỳ bạn có dùng trong Blockly hay không)
CONNECTOR_TO_PORT = {
    "A1,2": "AIN1",
    "A3,4": "AIN2",
    "A5,6": "AIN3",
    "A7,8": "AIN4",
    "D1,2": "DOUT1",
    "D3,4": "DOUT2",
    "D5,6": "DOUT3",
    "D7,8": "DOUT4",
    "UART": "UART",
    "I2C0": "I2C0",
    "I2C1": "I2C1",
}

# Servo: 1-pin (PWM)
SERVO_MAP = {
    "S_1": 45,
    "S_2": 47,
    "S_3": 48,
    "S_4": 40,
    "S_5": 41,
    "S_6": 42,
}

# =========================
# 2) Core utils (lazy import)
# =========================
__all__ = [
    "PORT_MAP", "SERVO_MAP",
    "port_pins", "port_pin", "servo_pin",
    "i2c", "uart_pins",
    "dht", "oled", "servo", "ultrasonic",
]

_mod_cache = {}
def _imp(mod_name):
    m = _mod_cache.get(mod_name)
    if m is None:
        m = __import__(mod_name)
        _mod_cache[mod_name] = m
    return m

def _norm_port(name):
    if name is None:
        return None
    s = str(name).strip()
    if not s:
        return s
    # cho phép user truyền "A1,2" / "D1,2" / ...
    s2 = CONNECTOR_TO_PORT.get(s, s)
    return str(s2).upper()

def port_pins(port):
    """
    Trả về (pin1, pin2) theo tên PORT (I2C0/I2C1/AIN1../DOUT1../UART).
    """
    p = _norm_port(port)
    if p not in PORT_MAP:
        raise ValueError("Unknown port: %s" % (port,))
    return PORT_MAP[p]

def port_pin(port, index=1):
    """
    Lấy 1 pin từ PORT 2-pin.
    index=1 -> pin1, index=2 -> pin2
    """
    a, b = port_pins(port)
    return a if int(index) == 1 else b

def servo_pin(s_port):
    """
    Trả về GPIO theo cổng servo dạng 'S_1'..'S_6'
    """
    p = _norm_port(s_port).replace("S", "S_") if isinstance(s_port, str) and s_port.upper().startswith("S") and "_" not in s_port else _norm_port(s_port)
    if p not in SERVO_MAP:
        raise ValueError("Unknown servo port: %s" % (s_port,))
    return SERVO_MAP[p]

def uart_pins():
    """Trả về (TX, RX) của cổng UART."""
    return PORT_MAP["UART"]

# =========================
# 3) BUS factories (cache)
# =========================
_i2c_cache = {}

def _i2c_id_from_port(port):
    p = _norm_port(port)
    # Theo các vệ tinh ext_dht/oled: i2c_id=0 => 8/9, i2c_id=1 => 10/11
    return 1 if p == "I2C1" else 0

def i2c(port="I2C0", freq=400_000):
    """
    Tạo (hoặc lấy cache) I2C cứng theo port:
      I2C0 => id 0, SDA=8 SCL=9
      I2C1 => id 1, SDA=10 SCL=11
    """
    p = _norm_port(port)
    if p not in ("I2C0", "I2C1"):
        raise ValueError("I2C port must be I2C0 or I2C1")
    key = (p, int(freq))
    obj = _i2c_cache.get(key)
    if obj is not None:
        return obj

    from machine import Pin, I2C
    sda, scl = PORT_MAP[p]
    bus_id = _i2c_id_from_port(p)
    obj = I2C(bus_id, scl=Pin(int(scl)), sda=Pin(int(sda)), freq=int(freq))
    _i2c_cache[key] = obj
    return obj

# =========================
# 4) Satellites facades
# =========================

class _DHTFacade:
    """
    Facade cho ext_dht.py
    - Lazy import ext_dht khi dùng
    - Hỗ trợ port name:
        + DHT20/AHT20: port I2C0/I2C1
        + DHT11: dùng 1 pin, có thể truyền port 2-pin và chọn index
    """
    def __init__(self):
        self._cache = {}
        self._default = None  # (sensor, port, kwargs_tuple)

    def init(self, sensor="dht20", port="I2C0", **kwargs):
        self._default = (str(sensor).lower(), _norm_port(port), tuple(sorted(kwargs.items())))
        return self._get(sensor=sensor, port=port, **kwargs)

    def _get(self, sensor="dht20", port="I2C0", **kwargs):
        s = str(sensor).lower()
        p = _norm_port(port)

        # DHT11: có thể dùng port 2-pin hoặc pin trực tiếp
        if s == "dht11":
            if "pin" not in kwargs:
                # nếu user truyền port 2-pin thì lấy pin1 mặc định
                pin_index = int(kwargs.pop("pin_index", 1))
                kwargs["pin"] = port_pin(p, pin_index) if p in PORT_MAP else int(p)
            key = ("dht11", kwargs.get("pin"))
            obj = self._cache.get(key)
            if obj is None:
                mod = _imp("ext_dht")
                obj = mod.DHT(sensor="dht11", **kwargs)
                self._cache[key] = obj
            return obj

        # DHT20/AHT20: luôn I2C
        if p not in ("I2C0", "I2C1"):
            raise ValueError("DHT20/AHT20 needs I2C port: I2C0 or I2C1")

        freq = int(kwargs.pop("freq", 100_000))
        addr = int(kwargs.pop("addr", 0x38))

        # tạo I2C object theo port, rồi đưa vào ext_dht để tránh ext_dht tự tạo lại
        bus = i2c(p, freq=freq)
        key = (s, p, freq, addr)
        obj = self._cache.get(key)
        if obj is None:
            mod = _imp("ext_dht")
            obj = mod.DHT(sensor=s, i2c=bus, addr=addr, freq=freq, **kwargs)
            self._cache[key] = obj
        return obj

    # ---- API ngắn gọn cho Blockly ----
    def read(self, sensor=None, port=None, **kwargs):
        obj = self._pick(sensor, port, **kwargs)
        return obj.read()

    def temperature(self, sensor=None, port=None, **kwargs):
        obj = self._pick(sensor, port, **kwargs)
        return obj.temperature()

    def humidity(self, sensor=None, port=None, **kwargs):
        obj = self._pick(sensor, port, **kwargs)
        return obj.humidity()

    def status(self, sensor=None, port=None, **kwargs):
        obj = self._pick(sensor, port, **kwargs)
        return obj.status()

    def _pick(self, sensor, port, **kwargs):
        if sensor is None and port is None and not kwargs and self._default is not None:
            s, p, kwt = self._default
            return self._get(sensor=s, port=p, **dict(kwt))
        return self._get(sensor=(sensor or "dht20"), port=(port or "I2C0"), **kwargs)


class _OledFacade:
    """
    Facade cho oled.py
    - Lazy import oled khi dùng
    - Hỗ trợ port I2C0/I2C1
    - Auto init lần đầu theo default
    """
    def __init__(self):
        self._cache = {}
        self._default_key = None

    def init(self, port="I2C0", **kwargs):
        dev = self._get(port=port, **kwargs)
        self._default_key = self._make_key(_norm_port(port), **kwargs)
        return dev

    def _make_key(self, port, **kwargs):
        width  = int(kwargs.get("width", 128))
        height = int(kwargs.get("height", 64))
        driver = str(kwargs.get("driver", "SSD1306"))
        addr   = kwargs.get("addr", None)
        freq   = int(kwargs.get("freq", 400_000))
        sh1106_offset = int(kwargs.get("sh1106_offset", kwargs.get("sh1106_col_offset", 2)))
        return (port, width, height, driver.upper(), addr, freq, sh1106_offset)

    def _get(self, port="I2C0", **kwargs):
        p = _norm_port(port)
        if p not in ("I2C0", "I2C1"):
            raise ValueError("OLED needs I2C port: I2C0 or I2C1")

        key = self._make_key(p, **kwargs)
        dev = self._cache.get(key)
        if dev is not None:
            return dev

        mod = _imp("oled")
        # oled.Oled wrapper dùng hard I2C; chọn i2c_id theo port
        i2c_id = _i2c_id_from_port(p)
        freq   = int(kwargs.get("freq", 400_000))
        width  = int(kwargs.get("width", 128))
        height = int(kwargs.get("height", 64))
        driver = kwargs.get("driver", kwargs.get("ctrl", "SSD1306"))
        addr   = kwargs.get("addr", None)
        sh1106_offset = int(kwargs.get("sh1106_offset", kwargs.get("sh1106_col_offset", 2)))

        dev = mod.Oled(width=width, height=height,
                      driver=driver, addr=addr,
                      i2c_id=i2c_id, freq=freq,
                      sh1106_offset=sh1106_offset)
        self._cache[key] = dev
        # nếu chưa có default -> set luôn
        if self._default_key is None:
            self._default_key = key
        return dev

    def _default(self):
        if self._default_key is None:
            self.init("I2C0")
        return self._cache[self._default_key]

    # ---- API ngắn gọn cho Blockly ----
    def clear(self):
        self._default().clear()

    def fill(self, c=0):
        self._default().fill(int(c))

    def show(self):
        self._default().show()

    def text(self, s, x=0, y=0, c=1):
        self._default().text(str(s), int(x), int(y), int(c))

    def text_wrap(self, s, x=0, y=0, c=1):
        self._default().text_wrap(str(s), int(x), int(y), int(c))

    def poweroff(self):
        self._default().poweroff()

    def poweron(self):
        self._default().poweron()

    def invert(self, inv=False):
        self._default().invert(bool(inv))

    def contrast(self, val):
        self._default().contrast(int(val))


class _ServoFacade:
    """
    Facade cho servo.py
    - Lazy import servo khi dùng
    - Nhận pin theo 'S_1'..'S_6' hoặc số GPIO
    """
    def _pin(self, p):
        if isinstance(p, str):
            return servo_pin(p)
        return int(p)

    def deg180(self, s_port_or_pin, angle, speed=100):
        mod = _imp("servo")
        mod.servo_180_to(self._pin(s_port_or_pin), angle, speed)

    def deg270(self, s_port_or_pin, angle, speed=100):
        mod = _imp("servo")
        mod.servo_270_to(self._pin(s_port_or_pin), angle, speed)

    def cont360(self, s_port_or_pin, speed_percent):
        mod = _imp("servo")
        mod.servo_360(self._pin(s_port_or_pin), speed_percent)

    def off(self, s_port_or_pin):
        mod = _imp("servo")
        mod.servo_off(self._pin(s_port_or_pin))


class _UltrasonicFacade:
    """
    Facade cho ultrasonic.py (HC-SR04)
    - Lazy import ultrasonic khi dùng
    - Nhận PORT 2-pin (DOUT1..DOUT4/AIN1..), map pin1=TRIG, pin2=ECHO
    """
    def __init__(self):
        self._cache = {}

    def init(self, port="DOUT1", **kwargs):
        p = _norm_port(port)
        key = (p, tuple(sorted(kwargs.items())))
        dev = self._cache.get(key)
        if dev is not None:
            return dev

        trig, echo = port_pins(p)
        mod = _imp("ultrasonic")
        dev = mod.HCSR04(trigger_pin=trig, echo_pin=echo, **kwargs)
        self._cache[key] = dev
        return dev

    def distance_cm(self, port="DOUT1", filter=True):
        dev = self.init(port)
        return dev.distance_cm(filter=bool(filter))

    def distance_mm(self, port="DOUT1", filter=True):
        dev = self.init(port)
        # ultrasonic.py đã có distance_mm() (có filter nội bộ distance_cm)
        if hasattr(dev, "distance_mm"):
            return dev.distance_mm()
        return int(dev.distance_cm(filter=bool(filter)) * 10)


# =========================
# 5) Public singletons (Blockly-friendly)
# =========================
dht = _DHTFacade()
oled = _OledFacade()
servo = _ServoFacade()
ultrasonic = _UltrasonicFacade()
