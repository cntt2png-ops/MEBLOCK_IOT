# boot.py - ESP32-S3 MicroPython boot script
# Simple boot with BLE UART REPL

import micropython
import os
import time
from machine import Pin

micropython.alloc_emergency_exception_buf(128)

# ===== Device name helper =====
try:
    from setting import load_device_name
except ImportError:
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
    print("[BOOT] BLE REPL failed:", e)

# ===== Optional: Double-Press BOOT (IO0) Detection =====
BOOT_PIN = 0
DOUBLE_BOOT_WINDOW_MS = 3000
DEBOUNCE_MS = 60

def check_double_boot():
    try:
        boot_btn = Pin(BOOT_PIN, Pin.IN, Pin.PULL_UP)
    except Exception as e:
        print("[BOOTKEY] Cannot init BOOT pin:", e)
        return False

    print("[BOOTKEY] Press BOOT (IO0) 2 times to enter recovery...")

    press_count = 0
    start_ms = time.ticks_ms()
    last_state = boot_btn.value()
    last_change_ms = start_ms

    while time.ticks_diff(time.ticks_ms(), start_ms) < DOUBLE_BOOT_WINDOW_MS:
        now = time.ticks_ms()
        state = boot_btn.value()

        if state != last_state and time.ticks_diff(now, last_change_ms) >= DEBOUNCE_MS:
            last_state = state
            last_change_ms = now

            if state == 0:
                press_count += 1
                print("[BOOTKEY] Press", press_count)

                if press_count >= 2:
                    print("[BOOTKEY] Double BOOT detected - entering recovery")
                    try:
                        os.remove("main.py")
                    except:
                        pass
                    return True

        time.sleep_ms(10)

    return False

if check_double_boot():
    print("[BOOT] Recovery mode - REPL only")
else:
    print("[BOOT] Normal boot")

print("[BOOT] Complete")