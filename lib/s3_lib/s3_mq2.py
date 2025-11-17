# mq2.py – Especially for ESP32-S3 DevKitC-1 (preset)
from machine import Pin, ADC
import time
import math

S3_MQ2_A = 1
S3_MQ2_D = 5


class MQ2:
    """
    Thư viện MQ-2 cho ESP32-S3 (MicroPython)
    - Hỗ trợ module 4 chân: VCC, GND, AOUT (A), DOUT (D)
    - Đọc ADC (analog), đọc ngõ ra số (digital)
    - Calibrate trong không khí sạch để tìm Ro
    - Tính Rs/Ro, ước lượng ppm và mức cảnh báo

    Nếu không chọn pin_a / pin_d thì sẽ dùng preset:
        pin_a = S3_MQ2_A
        pin_d = S3_MQ2_D
    """

    # Các hệ số đường cong log-log ước lượng từ datasheet MQ-2.
    # CHỈ MANG TÍNH THAM KHẢO, KHÔNG DÙNG CHO AN TOÀN CHÁY NỔ.
    # log10(Rs/Ro) = m * log10(ppm) + b  (xấp xỉ)
    GAS_CURVES = {
        "LPG":   {"m": -0.47, "b": 1.40},
        "CO":    {"m": -0.38, "b": 1.30},
        "SMOKE": {"m": -0.77, "b": 2.00},
    }

    def __init__(self, pin_a=None, pin_d=None, vref=3.3, adc_bits=12, samples=10):
        """
        pin_a : GPIO nối với chân AOUT (ngõ ra analog).
                Nếu None -> dùng S3_MQ2_A.
        pin_d : (tuỳ chọn) GPIO nối với chân DOUT (ngõ ra digital).
                Nếu None -> dùng S3_MQ2_D.
        vref  : điện áp tham chiếu cho ADC (ESP32 = 3.3V).
        adc_bits: độ phân giải ADC (ESP32 thường là 12 bit).
        samples: số mẫu trung bình mỗi lần đọc ADC.
        """
        if pin_a is None:
            pin_a = S3_MQ2_A
        if pin_d is None:
            pin_d = S3_MQ2_D

        # ADC
        self.adc = ADC(Pin(pin_a))
        try:
            # ESP32-S3: ATTN_11DB để đo ~0–3.3V, WIDTH_12BIT
            self.adc.atten(ADC.ATTN_11DB)
            self.adc.width(ADC.WIDTH_12BIT)
        except:
            # Nếu firmware không hỗ trợ các hàm trên thì bỏ qua
            pass

        # Ngõ ra digital (có thể không dùng)
        self.pin_d = Pin(pin_d, Pin.IN) if pin_d is not None else None

        self.vref = vref
        self.adc_max = (1 << adc_bits) - 1
        self.samples = samples

        # Các tham số mạch đo
        self.rl = 5.0      # kΩ, giá trị tải RL ~ 5k trong nhiều mạch MQ-2
        self.ro = None     # kΩ, sẽ được tính trong calibrate()

    def _read_adc(self):
        total = 0
        for _ in range(self.samples):
            total += self.adc.read()
            time.sleep_ms(5)
        return total // self.samples

    def read_raw(self):
        """
        Trả về giá trị ADC thô (0..adc_max).
        """
        return self._read_adc()

    def read_voltage(self):
        """
        Đọc điện áp tại chân AOUT (Volt).
        """
        return self.read_raw() * self.vref / self.adc_max

    def read_digital(self):
        """
        Đọc ngõ ra số DOUT (nếu có).
        Trả về:
            - 0 hoặc 1
            - None nếu không khởi tạo pin_d
        """
        if self.pin_d is None:
            return None
        return self.pin_d.value()

    def get_resistance(self):
        """
        Tính Rs (kΩ) từ điện áp đầu ra Vout.
        Sử dụng mạch phân áp: Vout = Vcc * RL / (Rs + RL)
        => Rs = RL * (Vcc - Vout) / Vout
        """
        vout = self.read_voltage()
        if vout <= 0:
            return 1e9   # tránh chia 0
        rs = self.rl * (self.vref - vout) / vout
        return rs

    def calibrate(self, clean_air_factor=9.83, duration=15):
        """
        Calibrate trong không khí sạch.

        clean_air_factor: tỉ lệ Rs/Ro trong không khí sạch (~9.83 cho MQ-2).
        duration (giây): thời gian đo trung bình.

        Sau khi calibrate, self.ro sẽ được gán (kΩ).
        """
        print(">> Calibrate MQ-2 trong không khí sạch (ESP32-S3)...")
        t0 = time.ticks_ms()
        rs_sum = 0.0
        count = 0

        while time.ticks_diff(time.ticks_ms(), t0) < duration * 1000:
            rs = self.get_resistance()
            rs_sum += rs
            count += 1
            time.sleep_ms(200)

        if count == 0:
            print("!! Calibrate thất bại: không đọc được mẫu nào")
            return None

        rs_air = rs_sum / count
        self.ro = rs_air / clean_air_factor
        print(">> Ro ≈ {:.2f} kΩ (Rs_air ≈ {:.2f} kΩ)".format(self.ro, rs_air))
        return self.ro

    def read_ratio(self):
        """
        Trả về Rs/Ro. Nếu chưa calibrate (Ro=None) -> None.
        """
        if self.ro is None:
            return None
        rs = self.get_resistance()
        return rs / self.ro

    def estimate_ppm(self, gas="SMOKE"):
        """
        Ước lượng nồng độ khí (ppm) theo loại gas:
            gas: "LPG", "CO", "SMOKE"
        Trả về:
            - ppm (float) hoặc None nếu chưa calibrate / gas không hỗ trợ.
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

        # log10(Rs/Ro) = m * log10(ppm) + b
        log_rs_ro = math.log10(rs_ro)
        m = curve["m"]
        b = curve["b"]

        log_ppm = (log_rs_ro - b) / m
        ppm = 10 ** log_ppm
        return ppm

    def get_level(self, gas="SMOKE", low_ppm=200, high_ppm=1000):
        """
        Trả về mức cảnh báo ("LOW", "MEDIUM", "HIGH") và ppm.
        Mặc định:
            LOW   : ppm < 200
            MEDIUM: 200 <= ppm < 1000
            HIGH  : ppm >= 1000
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

    def save_calibration(self, filename="mq2_ro.txt"):
        """
        Lưu Ro xuống file để dùng lại sau (trên cùng board).
        """
        if self.ro is None:
            print("!! Chưa có Ro để lưu. Gọi calibrate() trước.")
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
