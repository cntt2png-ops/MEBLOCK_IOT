# aiot_dht.py
# Hỗ trợ DHT20/AHT20 (I2C) và DHT11 (1-wire) cho MicroPython (ESP32/ESP32-S3)
# - Mặc định DHT20 dùng SoftI2C (ổn định, không cảnh báo deprecated)
# - Preset chân I2C theo board: "wroom" (SDA=21,SCL=22), "s3" (SDA=8,SCL=9)
# - API chung: read(), temperature(), humidity(), status()
# - Shim tương thích: read_dht20()/dht20_temperature()/... và tương tự cho DHT11

import time
from machine import Pin, I2C

try:
    from machine import SoftI2C as _SoftI2C
except ImportError:
    _SoftI2C = None

# DHT11: dùng module tích hợp của MicroPython (khuyến nghị)
try:
    from dht import DHT11 as _DHT11
except ImportError:
    _DHT11 = None  # Nếu None, sẽ báo lỗi rõ khi khởi tạo DHT11

# -------- Preset chân theo board (cho I2C / DHT20) --------
BOARD_PRESETS = {
    "wroom": {"sda": 21, "scl": 22},  # ESP32-WROOM DevKit
    "s3":    {"sda": 8,  "scl": 9},   # ESP32-S3 DevKitC-1
}

# =========================
# Driver nội bộ cho DHT20
# =========================
class _DHT20Driver:
    def __init__(self, *, board=None, sda=None, scl=None, i2c=None,
                 backend="soft", i2c_id=0, freq=100_000, addr=0x38):
        self.addr = addr
        self._t = None
        self._h = None
        self._last_ok = False

        # chọn chân
        preset_sda = preset_scl = None
        if board:
            b = board.lower()
            if b in BOARD_PRESETS:
                preset_sda = BOARD_PRESETS[b]["sda"]
                preset_scl = BOARD_PRESETS[b]["scl"]
            else:
                raise ValueError("Board không hỗ trợ preset: {}".format(board))
        _sda = sda if sda is not None else (preset_sda if preset_sda is not None else 21)
        _scl = scl if scl is not None else (preset_scl if preset_scl is not None else 22)

        # khởi tạo i2c nếu chưa có
        if i2c is None:
            be = backend.lower()
            if be == "auto":
                use_soft = (_SoftI2C is not None)
            elif be == "soft":
                if _SoftI2C is None:
                    raise RuntimeError("SoftI2C không khả dụng trên firmware này")
                use_soft = True
            elif be == "hard":
                use_soft = False
            else:
                raise ValueError('backend phải là "soft" | "hard" | "auto"')

            if use_soft:
                i2c = _SoftI2C(scl=Pin(_scl), sda=Pin(_sda), freq=freq)
            else:
                i2c = I2C(i2c_id, scl=Pin(_scl), sda=Pin(_sda), freq=freq)

        self.i2c = i2c
        self._ensure_calibrated()

    def _read_status(self) -> int:
        try:
            try:
                return self.i2c.readfrom_mem(self.addr, 0x71, 1)[0]
            except Exception:
                self.i2c.writeto(self.addr, bytes([0x71]))
                return self.i2c.readfrom(self.addr, 1)[0]
        except Exception:
            return 0x00

    def _ensure_calibrated(self):
        st = self._read_status()
        if (st & 0x08) == 0:  # chưa calibrated
            try:
                self.i2c.writeto(self.addr, bytes([0xBE, 0x08, 0x00]))
                time.sleep_ms(20)
            except Exception:
                pass
        start = time.ticks_ms()
        while True:
            st = self._read_status()
            if (st & 0x80) == 0:
                break
            if time.ticks_diff(time.ticks_ms(), start) > 200:
                break
            time.sleep_ms(5)

    def read(self) -> bool:
        try:
            self.i2c.writeto(self.addr, bytes([0xAC, 0x33, 0x00]))
        except Exception:
            self._last_ok = False
            return False

        start = time.ticks_ms()
        while True:
            st = self._read_status()
            if (st & 0x80) == 0:
                break
            if time.ticks_diff(time.ticks_ms(), start) > 150:
                break
            time.sleep_ms(5)

        try:
            data = self.i2c.readfrom(self.addr, 7)
        except Exception:
            self._last_ok = False
            return False

        if len(data) < 6:
            self._last_ok = False
            return False

        raw_h = (data[1] << 12) | (data[2] << 4) | (data[3] >> 4)
        raw_t = ((data[3] & 0x0F) << 16) | (data[4] << 8) | data[5]

        rh = (raw_h * 100.0) / 1048576.0
        t  = (raw_t * 200.0) / 1048576.0 - 50.0

        if 0.0 <= rh <= 100.0 and -50.0 <= t <= 85.0:
            self._h = rh
            self._t = t
            self._last_ok = True
        else:
            self._last_ok = False
        return self._last_ok

    def temperature(self):
        if self._t is None:
            self.read()
        return self._t

    def humidity(self):
        if self._h is None:
            self.read()
        return self._h

    def status(self):
        st = self._read_status()
        return {
            "busy": bool(st & 0x80),
            "calibrated": bool(st & 0x08),
            "last_ok": bool(self._last_ok),
        }

# =========================
# Driver nội bộ cho DHT11
# =========================
class _DHT11Driver:
    def __init__(self, *, pin=4):
        if _DHT11 is None:
            raise ImportError("Firmware thiếu module 'dht'. Vui lòng dùng MicroPython có sẵn dht.DHT11.")
        self._sensor = _DHT11(Pin(pin))
        self._t = None
        my_h = None
        self._h = None
        self._last_ok = False

    def read(self) -> bool:
        try:
            self._sensor.measure()
            self._t = self._sensor.temperature()
            self._h = self._sensor.humidity()
            self._last_ok = True
        except Exception:
            self._last_ok = False
        return self._last_ok

    def temperature(self):
        if self._t is None:
            self.read()
        return self._t

    def humidity(self):
        if self._h is None:
            self.read()
        return self._h

    def status(self):
        return {"last_ok": bool(self._last_ok)}

# =========================
# Lớp hợp nhất / mặt nạ API
# =========================
class DHT:
    def __init__(self, sensor="dht20", **kwargs):
        """
        sensor: "dht20" (AHT20/DHT20 qua I2C) | "dht11" (1-wire)
        - DHT20 kwargs: board, sda, scl, i2c, backend, i2c_id, freq, addr
        - DHT11 kwargs: pin
        """
        s = sensor.lower()
        if s in ("dht20", "aht20"):
            self.kind = "dht20"
            self.impl = _DHT20Driver(**kwargs)
        elif s == "dht11":
            self.kind = "dht11"
            # default chân 4 nếu không truyền
            if "pin" not in kwargs:
                kwargs["pin"] = 4
            self.impl = _DHT11Driver(**kwargs)
        else:
            raise ValueError('sensor phải là "dht20"|"aht20" hoặc "dht11"')

    # --- API chung ---
    def read(self) -> bool:
        return self.impl.read()

    def temperature(self):
        return self.impl.temperature()

    def humidity(self):
        return self.impl.humidity()

    def status(self):
        return self.impl.status()

    # --- Shim tương thích DHT20 ---
    def read_dht20(self): return self.read()
    def dht20_temperature(self): return self.temperature()
    def dht20_humidity(self): return self.humidity()

    # --- Shim tương thích DHT11 ---
    def read_dht11(self): return self.read()
    def dht11_temperature(self): return self.temperature()
    def dht11_humidity(self): return self.humidity()

# Alias tiện dụng
class DHT20(DHT):
    def __init__(self, **kwargs):
        super().__init__(sensor="dht20", **kwargs)

class DHT11(DHT):
    def __init__(self, **kwargs):
        super().__init__(sensor="dht11", **kwargs)
