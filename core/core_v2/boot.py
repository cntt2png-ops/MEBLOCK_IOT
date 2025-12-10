# boot.py - ESP32-S3 MicroPython boot script
# Simple boot with BLE UART REPL
#
# Features:
#   - BLE UART REPL (official MicroPython style)
#   - Optional: Double-Reset Detection for recovery

import micropython
import os

micropython.alloc_emergency_exception_buf(128)

# ===== Device name helper =====
try:
    from setting import load_device_name
except ImportError:
    # Fallback nếu chưa có setting.py
    def load_device_name():
        return "MEBLOCK-DEVICE"
    
# ===== BLE UART REPL =====
print("[BOOT] Starting BLE UART REPL...")

try:
    import ble_uart_repl
    device_name = load_device_name()
    print("[BOOT] BLE name:", device_name)
    ble_uart_repl.start(name=device_name)
    print("[BOOT] BLE UART REPL ready")
except Exception as e:
    print(f"[BOOT] BLE REPL failed: {e}")

# ===== Optional: Double-Reset Detection =====
DRD_WINDOW_MS = 5000
_DRD_FLAG = "/.drd"

def _exists(p):
    try: os.stat(p); return True
    except: return False

def _remove(p):
    try: os.remove(p); return True
    except: return False

def check_double_reset():
    if _exists(_DRD_FLAG):
        _remove(_DRD_FLAG)
        print("[DRD] Double reset detected - entering recovery")
        _remove("main.py")
        return True
    
    try:
        with open(_DRD_FLAG, "w") as f:
            f.write("1")
    except:
        pass
    
    from machine import Timer
    Timer(0).init(
        period=DRD_WINDOW_MS,
        mode=Timer.ONE_SHOT,
        callback=lambda t: micropython.schedule(lambda _: _remove(_DRD_FLAG), 0)
    )
    return False
if check_double_reset():
    print("[BOOT] Recovery mode - REPL only")
else:
    print("[BOOT] Normal boot")

print("[BOOT] Complete")