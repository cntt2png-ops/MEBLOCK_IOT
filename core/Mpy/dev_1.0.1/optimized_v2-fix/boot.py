# boot.py - ESP32-S3 MicroPython boot script
# Boot log uses onboard.send_inf/send_war/send_err when available.

import micropython
import os
import time
from machine import Pin

micropython.alloc_emergency_exception_buf(128)

# ===== Log helpers =====
try:
    from onboard import send_inf, send_war, send_err
except Exception:
    def _fallback_log(prefix, code_or_text, text=None):
        try:
            if text is None:
                print("[%s] %s" % (prefix, code_or_text))
            else:
                print("[%s] %s: %s" % (prefix, code_or_text, text))
        except Exception:
            pass

    def send_inf(code_or_text, text=None, out=None):
        _fallback_log("INF", code_or_text, text)

    def send_war(code_or_text, text=None, out=None):
        _fallback_log("WAR", code_or_text, text)

    def send_err(code_or_text, text=None, out=None):
        _fallback_log("ERR", code_or_text, text)

# ===== Device name helper =====
try:
    from setting import load_device_name
except ImportError:
    def load_device_name():
        return "MEBLOCK-DEVICE"

# ===== BLE UART REPL =====
send_inf("I_BOOT", "Starting BLE UART REPL")

try:
    import ble_uart_repl
    device_name = load_device_name()
    send_inf("I_BLE_NAME", device_name)
    ble_uart_repl.start(name=device_name)
    send_inf("I_BOOT", "BLE UART REPL ready")
except Exception as e:
    send_err("E_BLE_REPL", str(e))

# ===== Optional: Double-Press BOOT (IO0) Detection =====
BOOT_PIN = 0
DOUBLE_BOOT_WINDOW_MS = 3000
DEBOUNCE_MS = 60

def check_double_boot():
    try:
        boot_btn = Pin(BOOT_PIN, Pin.IN, Pin.PULL_UP)
    except Exception as e:
        send_war("W_BOOTKEY", "Cannot init BOOT pin: %s" % e)
        return False

    send_inf("I_BOOTKEY", "Press BOOT IO0 2 times to enter recovery")

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
                send_inf("I_BOOTKEY", "Press %d" % press_count)

                if press_count >= 2:
                    send_war("W_RECOVERY", "Double BOOT detected - entering recovery")
                    try:
                        os.remove("main.py")
                    except Exception:
                        pass
                    return True

        time.sleep_ms(10)

    return False

if check_double_boot():
    send_war("W_BOOT", "Recovery mode - REPL only")
else:
    send_inf("I_BOOT", "Normal boot")

send_inf("I_BOOT", "Complete")
