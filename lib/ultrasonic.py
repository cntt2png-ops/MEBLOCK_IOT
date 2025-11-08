# ultrasonic.py
# MicroPython HC-SR04 driver 
import time, machine
from machine import Pin

try:
    const
except NameError:  # CPython-type fallback (rare)
    def const(x): return x

_MAX_DISTANCE_CM = const(200)  # clamp to 2m để tránh outlier

def _normalize_key(k: str) -> str:
    # Chuẩn hóa key pin cho lookup (không phân biệt hoa thường, bỏ khoảng trắng)
    return str(k).strip().replace(" ", "").upper()

def _resolve_pin_id(spec, pinmap=None):
    """
    Chấp nhận:
      - int (vd 2) → dùng trực tiếp
      - '2' → 2
      - 'GPIO2', 'IO2', 'P2' → 2
      - String key có trong pinmap (vd 'TRIG', 'ECHO') → ánh xạ qua pinmap
    """
    if isinstance(spec, int):
        return spec

    if pinmap:
        k = _normalize_key(spec)
        # pinmap có thể đặt key tùy ý: 'TRIG', 'ECHO', 'D2', ...
        # hoặc dạng 'GPIO2' -> 2
        if k in { _normalize_key(x) for x in pinmap.keys() }:
            # tìm key khớp (không phân biệt hoa thường)
            for mk, mv in pinmap.items():
                if _normalize_key(mk) == k:
                    return int(mv)

    s = str(spec).strip().upper()
    # Số nguyên dạng chuỗi
    if s.isdigit():
        return int(s)

    # Các pattern phổ biến: GPIO2, IO2, P2
    for prefix in ("GPIO", "IO", "P"):
        if s.startswith(prefix) and s[len(prefix):].isdigit():
            return int(s[len(prefix):])

    raise ValueError("Không xác định được chân từ spec={!r}. Hãy truyền int/\"GPIOx\" hoặc cung cấp pinmap.".format(spec))


class HCSR04:
    """
    Driver HC-SR04 hỗ trợ khai báo chân linh hoạt & gán metadata board.

    Cách dùng (tối giản - tương thích bản cũ):
        sensor = HCSR04(trigger_pin=2, echo_pin=3)
        print(sensor.distance_cm())

    Dùng chuỗi chân:
        sensor = HCSR04(trigger_pin="GPIO2", echo_pin="IO3")

    Dùng pinmap tự đặt tên:
        pinmap = {"TRIG": 2, "ECHO": 3}
        sensor = HCSR04(trigger_pin="TRIG", echo_pin="ECHO", board="esp32-s3", pinmap=pinmap)

    Gợi ý tổ chức pinmap theo từng board:
        PINMAPS = {
            "ESP32-S3-DEVKITC-1": {"TRIG": 2, "ECHO": 3},
            "ESP32-WROOM-32":     {"TRIG": 5, "ECHO": 18},
        }
        sensor = HCSR04(trigger_pin="TRIG", echo_pin="ECHO",
                        board="ESP32-S3-DEVKITC-1", pinmap=PINMAPS.get("ESP32-S3-DEVKITC-1"))

    Tùy chọn pull:
        sensor = HCSR04("TRIG", "ECHO", pinmap=pinmap,
                        trigger_pull=None, echo_pull=Pin.PULL_DOWN)
    """
    def __init__(
        self,
        trigger_pin,
        echo_pin,
        *,
        echo_timeout_us=500*2*30,   # ~4m
        board=None,
        pinmap=None,
        trigger_pull=None,          # None / Pin.PULL_UP / Pin.PULL_DOWN
        echo_pull=None,             # None / Pin.PULL_UP / Pin.PULL_DOWN
        trigger_active_high=True    # Một số module cần đảo logic (hiếm)
    ):
        self.board = board
        self.pinmap = pinmap or {}
        self.echo_timeout_us = int(echo_timeout_us)
        self.trigger_active_high = bool(trigger_active_high)

        # Lịch sử để lọc (giữ ổn định API cũ)
        self._ars = []
        self._ats = []

        # Resolve pin id từ spec + pinmap
        trig_id = _resolve_pin_id(trigger_pin, self.pinmap)
        echo_id = _resolve_pin_id(echo_pin, self.pinmap)

        # Init pins
        self.trigger = Pin(trig_id, mode=Pin.OUT, pull=trigger_pull)
        self.echo    = Pin(echo_id, mode=Pin.IN,  pull=echo_pull)
        # Đặt mức nhàn rỗi cho trigger
        self.trigger.value(1 if not self.trigger_active_high else 0)

    def _send_pulse_and_wait(self):
        """
        Gửi xung 10us lên chân trigger và đo thời gian xung phản hồi trên echo.
        Dùng machine.time_pulse_us để đảm bảo chính xác micro giây.
        """
        # Ổn định cảm biến
        self.trigger.value(1 if not self.trigger_active_high else 0)
        time.sleep_us(5)

        # Phát xung 10us
        self.trigger.value(1 if self.trigger_active_high else 0)
        time.sleep_us(10)
        self.trigger.value(1 if not self.trigger_active_high else 0)

        try:
            pulse_time = machine.time_pulse_us(self.echo, 1, self.echo_timeout_us)
            return pulse_time
        except OSError as ex:
            # 110 = ETIMEDOUT trên nhiều port MicroPython
            if len(ex.args) > 0 and ex.args[0] == 110:
                raise OSError("Out of range")
            raise

    def distance_mm(self):
        return int(self.distance_cm() * 10)

    def distance_cm(self, filter=True):
        """
        Trả về khoảng cách (cm) dưới dạng float, có lọc nhiễu nhẹ theo cửa sổ thời gian.
        """
        pulse_time = self._send_pulse_and_wait()

        # Âm thanh đi & về: chia 2;  tốc độ âm thanh ~0.03432 cm/us => ~29.1 us/cm
        cms = (pulse_time / 2.0) / 29.1

        # Giới hạn để tránh outlier
        if cms < 0 or cms > _MAX_DISTANCE_CM:
            cms = _MAX_DISTANCE_CM

        if not filter:
            return cms

        # Bộ lọc ổn định (same as previous version, giữ tương thích)
        self._ars.append(cms)
        self._ats.append(time.time_ns())
        if len(self._ars) > 5:
            self._ars.pop(0)
            self._ats.pop(0)

        # Giữ cửa sổ trong ~0.5s
        while self._ats and (self._ats[-1] - self._ats[0] > 5e8):
            self._ars.pop(0)
            self._ats.pop(0)

        # Nếu chưa đủ mẫu, đo thêm một nhịp ngắn
        if len(self._ars) < 2:
            time.sleep_ms(30)
            pulse_time = self._send_pulse_and_wait()
            cms2 = (pulse_time / 2.0) / 29.1
            if cms2 < 0 or cms2 > _MAX_DISTANCE_CM:
                cms2 = _MAX_DISTANCE_CM
            self._ars.append(cms2)
            self._ats.append(time.time_ns())

        # Chọn giá trị "ổn" (tương tự bản cũ)
        N = len(self._ars)
        Fi = [1] * N
        Fd = [1] * N
        maxd = vald = 0
        for i in range(N):
            for j in range(i):
                if (self._ars[i] >= self._ars[j]) and (self._ars[i] - self._ars[j]) < 10:
                    Fi[i] = max(Fi[i], Fi[j] + 1)
                if (self._ars[i] <= self._ars[j]) and (self._ars[j] - self._ars[i]) < 10:
                    Fd[i] = max(Fd[i], Fd[j] + 1)
                if maxd < Fi[i] or maxd < Fd[i]:
                    maxd = max(Fi[i], Fd[i])
                    vald = self._ars[i]

        if maxd <= N / 2:
            vald = sum(self._ars) / N

        return round(vald * 10) / 10
