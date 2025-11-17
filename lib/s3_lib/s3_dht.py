# dht.py – Especially for ESP32-S3 DevKitC-1 (preset)

import time
from machine import Pin, I2C

try:
    from machine import SoftI2C as _SoftI2C
except ImportError:
    _SoftI2C = None

try:
    from dht import DHT11 as _DHT11
except ImportError:
    _DHT11 = None

# ===== PRESET CHÂN S3 DEVKIT C1 =====
S3_I2C_SDA = 8
S3_I2C_SCL = 9
S3_DHT11_PIN = 4


class _DHT20Driver:
    def __init__(self, *, sda=None, scl=None, i2c=None,
                 backend="soft", i2c_id=0, freq=100_000, addr=0x38):
        self.addr = addr
        self._t = None
        self._h = None
        self._last_ok = False

        # nếu không truyền thì dùng preset S3
        _sda = sda if sda is not None else S3_I2C_SDA
        _scl = scl if scl is not None else S3_I2C_SCL

        # khởi tạo I2C
        if i2c is None:
            be = backend.lower()
            if be == "auto":
                use_soft = (_SoftI2C is not None)
            elif be == "soft":
                if _SoftI2C is None:
                    raise RuntimeError("Firmware is not SoftI2C Supportted")
                use_soft = True
            elif be == "hard":
                use_soft = False
            else:
                raise ValueError('backend must be "soft" | "hard" | "auto"')

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
        if (st & 0x08) == 0:
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


class _DHT11Driver:
    def __init__(self, *, pin=None):
        if _DHT11 is None:
            raise ImportError("cannot find module 'dht'in this firmware")
        _pin = pin if pin is not None else S3_DHT11_PIN
        self._sensor = _DHT11(Pin(_pin))
        self._t = None
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


class DHT:
    def __init__(self, sensor="dht20", **kwargs):
        """
        sensor: "dht20" | "aht20" | "dht11"
        ESP32-S3 DevKitC-1 preset:
            - DHT20: I2C SDA=8, SCL=9 nếu không đặt chân
            - DHT11: GPIO4 nếu không truyền
        """
        s = sensor.lower()
        if s in ("dht20", "aht20"):
            self.kind = "dht20"
            self.impl = _DHT20Driver(**kwargs)
        elif s == "dht11":
            self.kind = "dht11"
            self.impl = _DHT11Driver(**kwargs)
        else:
            raise ValueError('sensor must be "dht20"|"aht20" hoac "dht11"')

    def read(self) -> bool:
        return self.impl.read()

    def temperature(self):
        return self.impl.temperature()

    def humidity(self):
        return self.impl.humidity()

    def status(self):
        return self.impl.status()

    # Shim
    def read_dht20(self): return self.read()
    def dht20_temperature(self): return self.temperature()
    def dht20_humidity(self): return self.humidity()

    def read_dht11(self): return self.read()
    def dht11_temperature(self): return self.temperature()
    def dht11_humidity(self): return self.humidity()


class DHT20(DHT):
    def __init__(self, **kwargs):
        super().__init__(sensor="dht20", **kwargs)


class DHT11(DHT):
    def __init__(self, **kwargs):
        super().__init__(sensor="dht11", **kwargs)
