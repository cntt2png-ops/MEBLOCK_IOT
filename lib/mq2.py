# mq2.py
from machine import Pin, ADC
import time
import math

class MQ2:
    """
    Thư viện MQ-2 cho ESP32-S3 (MicroPython)
    - Hỗ trợ module 4 chân: VCC, GND, AOUT (A), DOUT (D)
    - Đọc ADC (analog), đọc ngõ ra số (digital)
    - Calibrate trong không khí sạch để tìm Ro
    - Tính Rs/Ro, ước lượng ppm và mức cảnh báo
    """

    # Các hệ số đường cong log-log ước lượng từ datasheet MQ-2.
    # CHỈ MANG TÍNH THAM KHẢO, KHÔNG DÙNG CHO AN TOÀN CHÁY NỔ.
    GAS_CURVES = {
        # log10(Rs/Ro) = m * log10(ppm) + b  (xấp xỉ)
        "LPG":   {"m": -0.47, "b": 1.40},
        "CO":    {"m": -0.38, "b": 1.30},
        "SMOKE": {"m": -0.77, "b": 2.00},
    }

    def __init__(self, pin_a, pin_d=None, vref=3.3, adc_bits=12, samples=10):
        """
        pin_a : GPIO nối với chân AOUT (ngõ ra analog)
        pin_d : (tuỳ chọn) GPIO nối với chân DOUT (ngõ ra digital)
        vref  : điện áp tham chiếu cho ADC (ESP32 = 3.3V)
        adc_bits: độ phân giải ADC (ESP32 thường là 12 bit)
        samples : số mẫu đọc để lấy trung bình (lọc nhiễu)
        """
        self.adc = ADC(Pin(pin_a))
        try:
            self.adc.atten(ADC.ATTN_11DB)       # full range ~0 - 3.3V
            self.adc.width(ADC.WIDTH_12BIT)
        except:
            # Một số port MicroPython có thể không hỗ trợ các hàm này
            pass

        self.pin_d = Pin(pin_d, Pin.IN) if pin_d is not None else None

        self.vref = vref
        self.adc_max = (1 << adc_bits) - 1
        self.samples = samples

        self.rl = 5.0      # kΩ, điện trở tải thường trên module MQ-2 là 5k
        self.ro = None     # kΩ, điện trở trong không khí sạch sau khi calibrate

    # ----------- Các hàm đọc cơ bản -----------

    def _read_adc(self):
        total = 0
        for _ in range(self.samples):
            total += self.adc.read()
            time.sleep_ms(5)
        return total // self.samples

    def read_raw(self):
        """Giá trị ADC thô (0..4095)."""
        return self._read_adc()

    def read_voltage(self):
        """Điện áp tương ứng tại chân AOUT (V)."""
        return self.read_raw() * self.vref / self.adc_max

    def read_digital(self):
        """
        Đọc ngõ ra DOUT.
        Trả về:
            - 0: thường là vượt ngưỡng (gas nhiều, LED module sáng)
            - 1: dưới ngưỡng
            - None: nếu không nối pin_d
        """
        if self.pin_d is None:
            return None
        return self.pin_d.value()

    # ----------- Điện trở Rs, Ro & calibrate -----------

    def get_resistance(self):
        """
        Tính Rs (kΩ) dựa vào phân áp:
        Vout = Vref * RL / (RL + Rs)
        => Rs = RL * (Vref - Vout) / Vout
        """
        vout = self.read_voltage()
        if vout <= 0:
            return 1e9  # tránh chia 0, coi như vô cùng lớn
        rs = self.rl * (self.vref - vout) / vout
        return rs

    def calibrate(self, clean_air_factor=9.83, duration=15):
        """
        Calibrate cảm biến trong KHÔNG KHÍ SẠCH:
        - clean_air_factor ~ 9.83 cho MQ-2 (theo datasheet)
        - duration: thời gian lấy mẫu (giây) để tính trung bình Rs_air

        Sau khi calibrate, thuộc tính self.ro sẽ được gán (kΩ).
        """
        print(">> Bắt đầu calibrate MQ-2 trong không khí sạch...")
        print("   Đảm bảo không có khói, gas quanh cảm biến.")
        t0 = time.ticks_ms()
        rs_sum = 0.0
        count = 0

        while time.ticks_diff(time.ticks_ms(), t0) < duration * 1000:
            rs = self.get_resistance()
            rs_sum += rs
            count += 1
            time.sleep_ms(200)

        if count == 0:
            print("!! Calibrate thất bại: không đọc được giá trị.")
            return None

        rs_air = rs_sum / count
        self.ro = rs_air / clean_air_factor
        print(">> Calibrate xong. Ro ≈ {:.2f} kΩ (Rs_air ≈ {:.2f} kΩ)".format(self.ro, rs_air))
        return self.ro

    def read_ratio(self):
        """
        Tỷ lệ Rs/Ro:
        - Nếu chưa calibrate (Ro=None) → trả về None.
        - Giá trị càng NHỎ → nồng độ khí càng CAO.
        """
        if self.ro is None:
            return None
        rs = self.get_resistance()
        return rs / self.ro

    # ----------- Ước lượng nồng độ ppm -----------

    def estimate_ppm(self, gas="SMOKE"):
        """
        Ước lượng nồng độ ppm cho một loại khí:
            gas: "LPG", "CO", "SMOKE"

        Công thức:
            log10(Rs/Ro) = m * log10(ppm) + b
            => log10(ppm) = (log10(Rs/Ro) - b) / m

        Trả về:
            - ppm (float) nếu có đủ dữ liệu
            - None nếu chưa calibrate hoặc không có đường cong khí đó

        CẢNH BÁO: chỉ mang tính tham khảo học tập,
        KHÔNG dùng cho hệ thống an toàn thực tế.
        """
        if self.ro is None:
            return None

        gas = gas.upper()
        curve = self.GAS_CURVES.get(gas)
        if curve is None:
            return None

        rs_ro = self.read_ratio()
        if rs_ro is None or rs_ro <= 0:
            return None

        log_rs_ro = math.log10(rs_ro)
        m = curve["m"]
        b = curve["b"]

        log_ppm = (log_rs_ro - b) / m
        ppm = 10 ** log_ppm
        return ppm

    def get_level(self, gas="SMOKE", low_ppm=200, high_ppm=1000):
        """
        Phân mức cảnh báo theo ppm:
            - ppm <  low_ppm  → 'LOW'
            - low_ppm..high_ppm → 'MEDIUM'
            - > high_ppm → 'HIGH'
        Trả về:
            - ('LOW'/'MEDIUM'/'HIGH', ppm) hoặc (None, None) nếu không đo được.
        """
        ppm = self.estimate_ppm(gas)
        if ppm is None:
            return None, None

        if ppm < low_ppm:
            level = "LOW"
        elif ppm < high_ppm:
            level = "MEDIUM"
        else:
            level = "HIGH"

        return level, ppm

    # ----------- Lưu / đọc Ro vào file -----------

    def save_calibration(self, filename="mq2_ro.txt"):
        """
        Lưu giá trị Ro vào file để lần sau khỏi calibrate lại.
        """
        if self.ro is None:
            print("!! Chưa có Ro để lưu. Hãy calibrate trước.")
            return False
        try:
            with open(filename, "w") as f:
                f.write(str(self.ro))
            print(">> Đã lưu Ro = {:.2f} kΩ vào '{}'".format(self.ro, filename))
            return True
        except Exception as e:
            print("!! Lỗi khi lưu Ro:", e)
            return False

    def load_calibration(self, filename="mq2_ro.txt"):
        """
        Đọc giá trị Ro từ file (nếu có).
        """
        try:
            with open(filename, "r") as f:
                content = f.read().strip()
                self.ro = float(content)
            print(">> Đã tải Ro từ file: Ro ≈ {:.2f} kΩ".format(self.ro))
            return self.ro
        except Exception as e:
            print("!! Không đọc được file Ro:", e)
            return None
