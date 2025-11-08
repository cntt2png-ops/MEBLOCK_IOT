# boot.py — ESP32-WROOM-32 (MicroPython 1.26.x)
# - Double-Reset: bấm RESET 2 lần trong 5 giây => xóa main.py
# - Không dùng LED/NeoPixel, chỉ in log
# - Khởi động BLE (nếu DEV_VERSION >= 4)
# - main.py sẽ tự chạy sau khi boot.py kết thúc

import os, time, gc, sys
import esp
from machine import Timer
from micropython import schedule
from setting import *
from utility import *

# ===== Cấu hình (có thể override qua setting.py) =====
DRD_TIMEOUT_MS = 5000
DEV_VERSION = 0
VERSION = "0.0.0"

# Giảm log ROM
try:
    esp.osdebug(None)
except Exception:
    pass

# ===== Import setting/utility nếu có =====
try:
    from setting import *  # DEV_VERSION, VERSION, ...
except Exception as e:
    print("[BOOT] setting.py not loaded:", e)

try:
    from utility import *
except Exception as e:
    print("[BOOT] utility.py not loaded:", e)

# ===== Double-Reset Detector với NVS =====
try:
    from esp32 import NVS
    _nvs = NVS("drd")
except Exception:
    _nvs = None

def _is_armed():
    if _nvs is None:
        return False
    try:
        return _nvs.get_i32("armed") == 1
    except OSError:
        return False

def _arm():
    if _nvs is None:
        return
    try:
        _nvs.set_i32("armed", 1)
        _nvs.commit()
    except Exception as e:
        print("[DRD] arm failed:", e)

def _disarm():
    if _nvs is None:
        return
    try:
        _nvs.erase_key("armed")
        _nvs.commit()
    except Exception as e:
        print("[DRD] disarm failed:", e)

# ===== Xử lý DRD =====
if _is_armed():
    print("[DRD] Double-reset detected -> RECOVERY")
    print("[DRD] Remove main.py if exists ...")
    try:
        os.remove("main.py")
        print("[DRD] main.py removed.")
    except OSError:
        print("[DRD] main.py not found (skip).")
    _disarm()
else:
    _arm()
    print("[DRD] Armed. Press RESET again within %.1f s for recovery..." % (DRD_TIMEOUT_MS / 1000))

    _t = None
    def _disarm_scheduled(_):
        # Hàm chạy ở context an toàn (không phải ISR)
        print("[DRD] Window expired -> disarmed.")
        _disarm()
        try:
            if _t:
                _t.deinit()
        except Exception:
            pass

    def _timer_isr(t):
        # ISR: chỉ schedule công việc nặng ra thread chính
        try:
            schedule(_disarm_scheduled, 0)
        except Exception:
            pass

    try:
        # ESP32: dùng timer phần cứng 0..3. Chọn 1 để tránh va chạm.
        _t = Timer(1)
        _t.init(mode=Timer.ONE_SHOT, period=DRD_TIMEOUT_MS, callback=_timer_isr)
    except Exception as e:
        # Fallback: nếu Timer còn lỗi (ValueError...), chờ blocking rồi disarm
        print("[DRD] HW Timer init failed:", e)
        time.sleep_ms(DRD_TIMEOUT_MS)
        _disarm()
        print("[DRD] Window expired -> disarmed. (blocking fallback)")

# ===== Tiện ích load module =====
def stop_all():
    pass

def run(mod):
    if mod in sys.modules:
        del sys.modules[mod]
    __import__(mod)

# ===== Khởi động BLE (nếu hỗ trợ) =====
while True:
    try:
        if DEV_VERSION >= 4:
            try:
                run("bleuart")
                run("blerepl")
                run("ble")
                from ble import ble_o, ble
                ble.start()
                print("[BLE] Started.")
            except Exception as err:
                print("[BLE] Failed to start:", err)
        else:
            print("[BLE] This device does not support Bluetooth.")
        gc.collect()
        print("[BOOT] Firmware version:", VERSION)
        break
    except KeyboardInterrupt:
        print("[BOOT] Device is booting...")

# ===== Thông báo về main.py =====
try:
    os.stat("main.py")
    print("[BOOT] main.py found -> it will run after boot.py exits.")
except OSError:
    print("[BOOT] No main.py -> no user program to run.")

print("[BOOT] Done.")
