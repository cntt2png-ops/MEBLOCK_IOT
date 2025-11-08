# ble.py — BLE wrapper + PUT/DATA/DONE (patched: no slice deletion on bytearray)
from machine import reset
import os
import ubinascii
from bleuart import BLEUART

try:
    from setting import BLE_NAME
except Exception:
    BLE_NAME = "ESP32-BLE-OTA"

def _safe_decode(b):
    # MicroPython không hỗ trợ errors="ignore" trong .decode()
    try:
        return b.decode()
    except Exception:
        try:
            return b.decode("latin-1")
        except Exception:
            # thay byte lạ bằng '.'
            return "".join(chr(x) if 32 <= x < 127 else "." for x in b)

class BLEMain:
    def __init__(self, name=BLE_NAME):
        self.name = name
        self.uart = None
        self._rx_buf = bytearray()
        self._put = None
        self._started = False

    def _on_rx(self, data):
        if not data:
            return
        # Ghép buffer, tách theo \n — KHÔNG dùng `del` trên bytearray
        self._rx_buf.extend(data)
        # Cắt lần lượt từng dòng kết thúc bằng '\n'
        while True:
            i = self._rx_buf.find(b"\n")
            if i < 0:
                break
            line_b = bytes(self._rx_buf[:i])    # copy phần dòng ra bytes
            self._rx_buf = self._rx_buf[i+1:]   # gán lại phần còn lại (không xóa slice)
            line = _safe_decode(line_b).strip()
            self._handle_line(line)

        # Chặn trường hợp peer gửi quá dài mà không có '\n'
        if len(self._rx_buf) > 4096:
            # tránh chiếm RAM vô hạn
            self._rx_buf = self._rx_buf[-1024:]

    def _handle_line(self, line):
        print("[BLE][RX]:", line)
        low = line.lower()

        # 1) ECHO/PING
        if low == "ping":
            self.uart.send("PONG\n"); return
        if low.startswith("echo "):
            self.uart.send(line[5:] + "\n"); return

        # 2) LS
        if low == "ls":
            try:
                self.uart.send("FILES " + ",".join(os.listdir()) + "\n")
            except Exception as e:
                self.uart.send("ERR LS %s\n" % e)
            return

        # 3) RESET
        if low == "reset":
            self.uart.send("OK RESET\n"); reset(); return

        # 4) PUT <name> <size>
        if low.startswith("put "):
            try:
                _, name, size_s = line.split(None, 2)
                size = int(size_s)
                # đóng phiên cũ nếu còn
                if self._put and self._put.get("fp"):
                    try: self._put["fp"].close()
                    except: pass
                self._put = {"name": name, "left": size, "fp": open(name, "wb")}
                self.uart.send("OK PUT %s %d\n" % (name, size))
            except Exception as e:
                self._put = None
                self.uart.send("ERR PUT %s\n" % e)
            return

        # 5) DATA <base64>
        if low.startswith("data "):
            if not self._put or not self._put.get("fp"):
                self.uart.send("ERR DATA NOSESSION\n"); return
            try:
                b = ubinascii.a2b_base64(line[5:])
                self._put["fp"].write(b)
                self._put["left"] -= len(b)
                if self._put["left"] < 0:
                    self._put["left"] = 0
                # QUAN TRỌNG: phản hồi ACK để PC cập nhật tiến trình
                self.uart.send("OK %d\n" % self._put["left"])
            except Exception as e:
                self.uart.send("ERR DATA %s\n" % e)
            return

        # 6) DONE
        if low == "done":
            if self._put and self._put.get("fp"):
                try:
                    self._put["fp"].close()
                    name, left = self._put["name"], self._put["left"]
                    self._put = None
                    if left == 0:
                        self.uart.send("OK SAVED\n")
                    else:
                        self.uart.send("WARN LEFT %d\n" % left)
                except Exception as e:
                    self.uart.send("ERR DONE %s\n" % e)
            else:
                self.uart.send("ERR DONE NOSESSION\n")
            return

        self.uart.send("ERR UNKNOWN\n")

    def start(self):
        if self._started:
            print("[BLE] Started.")
            return
        try:
            import ubluetooth  # đảm bảo FW có BLE
        except Exception as e:
            print("[BLE] Firmware missing ubluetooth:", e)
            return
        try:
            self.uart = BLEUART(name=self.name, rx_callback=self._on_rx)
            self._started = True
            print("[BLE] Started.")
        except Exception as e:
            print("[BLE] Start error:", e)

ble_o = BLEMain()
ble   = ble_o
