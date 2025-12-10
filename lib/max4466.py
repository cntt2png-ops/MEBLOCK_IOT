# max4466.py
from machine import ADC, Pin
import time

class Max4466:
    def __init__(self, board="s3", pin=None, atten=ADC.ATTN_11DB):
        """
        board: tên board (để sau này mapping chân, tạm chưa dùng)
        pin: chân GPIO nối OUT của MAX4466 (bắt buộc phải truyền)
        atten: hệ số suy giảm ADC (ATTN_11DB đo được ~0-3.3V)
        """
        if pin is None:
            raise ValueError("Bạn phải truyền pin ADC nối với MAX4466 (vd pin=1).")

        self.board = board
        self.adc = ADC(Pin(pin))
        self.adc.atten(atten)

        self.baseline = 0

        # Biến cho clap / double clap
        self.last_clap = 0
        self._waiting_second = False
        self._first_clap_time = 0

        self.calibrate()

    def calibrate(self, samples=500, delay_us=200):
        """
        Lấy giá trị trung bình (baseline) trong môi trường yên tĩnh.
        Gọi lúc khởi tạo, hoặc khi muốn calibrate lại.
        """
        total = 0
        for _ in range(samples):
            total += self.adc.read()
            if delay_us:
                time.sleep_us(delay_us)
        self.baseline = total // samples

    def read_raw(self):
        """
        Đọc giá trị ADC thô (0–4095).
        """
        return self.adc.read()

    def read_level(self, samples=80, delay_us=200):
        """
        Đo “độ to” (biên độ) âm thanh trong 1 cửa sổ ngắn.

        Trả về: biên độ (max |sample - baseline|) trong khoảng samples mẫu.
        """
        max_diff = 0
        for _ in range(samples):
            val = self.adc.read()
            diff = abs(val - self.baseline)
            if diff > max_diff:
                max_diff = diff
            if delay_us:
                time.sleep_us(delay_us)
        return max_diff

    def _detect_single_clap(self, threshold, min_interval_ms):
        """
        Hàm nội bộ: phát hiện 1 “xung” vỗ tay mới (level > threshold)
        và cách lần trước ít nhất min_interval_ms.
        """
        level = self.read_level()
        now = time.ticks_ms()

        if level > threshold and time.ticks_diff(now, self.last_clap) > min_interval_ms:
            self.last_clap = now
            return True, now, level

        return False, now, level

    def is_clap(self, threshold=800, min_interval_ms=300):
        """
        Phát hiện 1 vỗ tay đơn.
        """
        clap, _, _ = self._detect_single_clap(threshold, min_interval_ms)
        return clap

    def is_double_clap(self,
                        threshold=800,
                        min_interval_ms=80,
                        max_gap_ms=450):
        """
        Phát hiện vỗ tay 2 lần liên tiếp (double clap).

        - threshold: ngưỡng biên độ để coi là vỗ tay.
        - min_interval_ms: thời gian tối thiểu giữa 2 lần đọc “clap”
          (lọc rung/nhiễu rất sát nhau, vd 50–80 ms).
        - max_gap_ms: khoảng cách tối đa giữa 2 vỗ tay để coi là "double"
          (vd 450 ms ~ phù hợp với clap-clap cách 200–300 ms).

        Trả về True đúng 1 lần khi vừa nhận đủ 2 vỗ tay trong cửa sổ thời gian.
        """
        now = time.ticks_ms()

        # Nếu đang chờ clap thứ 2 mà quá lâu thì hủy trạng thái chờ
        if self._waiting_second and time.ticks_diff(now, self._first_clap_time) > max_gap_ms:
            self._waiting_second = False

        # Thử phát hiện 1 clap mới
        clap, now, level = self._detect_single_clap(threshold, min_interval_ms)

        if not clap:
            return False

        # Nếu đây là clap đầu tiên trong cặp
        if not self._waiting_second:
            self._waiting_second = True
            self._first_clap_time = now
            # Chưa đủ double, chỉ mới clap #1
            return False

        # Đang chờ clap thứ 2
        gap = time.ticks_diff(now, self._first_clap_time)

        if gap <= max_gap_ms:
            # Đủ 2 clap trong khoảng thời gian cho phép → double clap
            self._waiting_second = False
            return True

        # Clap tới quá muộn: coi như clap mới, bắt đầu chuỗi mới
        self._first_clap_time = now
        return False
