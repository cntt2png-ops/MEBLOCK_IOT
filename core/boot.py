# boot.py - MEBLOCK MicroPython boot script (ESP32-S3/C3)
# - Start BLE UART REPL
# - Double-reset detector (DRD) giống logic core C++:
#   * Chỉ "arm" DRD khi POWERON hoặc HARD reset
#   * SOFT reset (Ctrl+D / raw REPL reset) sẽ không kích hoạt DRD
#   * Không xóa main.py (tránh mất file). Khi DRD kích hoạt → vào "recovery"
#     và CHẶN chạy main.py bằng SystemExit (REPL-only).

import micropython
import os

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


# ===== DRD (Double Reset Detector) =====
DRD_TIMEOUT_MS = 8000  # giống core C++
DRD_MAGIC_I32 = -559038737  # 0xDEADBEEF dưới dạng signed int32 (NVS dùng i32)
_DRD_FLAG_FILE = "/.drd"
_drd_timer = None

# Prefer NVS (giống Preferences bên C++). Nếu không có NVS thì fallback sang file.
_NVS = None
_DRD_BACKEND = "file"
try:
    import esp32
    _NVS = esp32.NVS("drd")
    _DRD_BACKEND = "nvs"
except Exception:
    _NVS = None
    _DRD_BACKEND = "file"


def _reset_cause_name(c):
    try:
        import machine
        m = machine
        if c == m.PWRON_RESET:
            return "PWRON_RESET"
        if c == m.HARD_RESET:
            return "HARD_RESET"
        if hasattr(m, "SOFT_RESET") and c == m.SOFT_RESET:
            return "SOFT_RESET"
        if hasattr(m, "WDT_RESET") and c == m.WDT_RESET:
            return "WDT_RESET"
        if hasattr(m, "DEEPSLEEP_RESET") and c == m.DEEPSLEEP_RESET:
            return "DEEPSLEEP_RESET"
    except Exception:
        pass
    return str(c)


def _file_exists(p):
    try:
        os.stat(p)
        return True
    except Exception:
        return False


def _file_remove(p):
    try:
        os.remove(p)
    except Exception:
        pass


def _drd_get_flag():
    if _DRD_BACKEND == "nvs" and _NVS:
        try:
            return _NVS.get_i32("flag")
        except Exception:
            return 0
    # file backend
    return 1 if _file_exists(_DRD_FLAG_FILE) else 0


def _drd_set_flag(v):
    if _DRD_BACKEND == "nvs" and _NVS:
        try:
            _NVS.set_i32("flag", int(v))
            _NVS.commit()
        except Exception:
            pass
        return
    # file backend
    if int(v) != 0:
        try:
            with open(_DRD_FLAG_FILE, "w") as f:
                f.write("1")
        except Exception:
            pass
    else:
        _file_remove(_DRD_FLAG_FILE)


def _drd_clear_scheduled(_):
    _drd_set_flag(0)


def _drd_arm_clear_timer():
    global _drd_timer
    try:
        from machine import Timer
        _drd_timer = Timer(0)
        _drd_timer.init(
            period=DRD_TIMEOUT_MS,
            mode=Timer.ONE_SHOT,
            callback=lambda t: micropython.schedule(_drd_clear_scheduled, 0),
        )
    except Exception:
        # Fail-safe: nếu Timer không chạy, cờ sẽ bị clear ở lần reset không phù hợp
        pass


def check_double_reset():
    try:
        import machine
        cause = machine.reset_cause()
        print("[DRD] backend =", _DRD_BACKEND, "| reset_cause =", cause, _reset_cause_name(cause))

        # Chỉ arm DRD cho POWERON hoặc HARD reset (giống POWERON/EXT bên C++).
        if cause not in (machine.PWRON_RESET, machine.HARD_RESET):
            _drd_set_flag(0)
            print("[DRD] Not PWRON/HARD -> skip DRD & clear flag")
            return False

        # Nếu đang dùng NVS thì dọn file /.drd cũ (từ bản trước) để tránh rác
        if _DRD_BACKEND == "nvs":
            _file_remove(_DRD_FLAG_FILE)

        flag = _drd_get_flag()
        if (_DRD_BACKEND == "nvs" and flag == DRD_MAGIC_I32) or (_DRD_BACKEND == "file" and flag == 1):
            _drd_set_flag(0)
            print("[DRD] Double reset detected -> RECOVERY (REPL only)")
            return True

        # First reset -> arm window
        _drd_set_flag(DRD_MAGIC_I32 if _DRD_BACKEND == "nvs" else 1)
        _drd_arm_clear_timer()
        print("[DRD] First hard reset -> arm window", DRD_TIMEOUT_MS, "ms")
        return False

    except Exception as e:
        print("[DRD] Error:", e)
        return False


_RECOVERY = check_double_reset()
if _RECOVERY:
    print("[BOOT] Recovery mode: skip running main.py (no file deletion).")
    # Chặn main.py chạy: boot.py kết thúc bằng SystemExit -> vào REPL (dupterm BLE vẫn hoạt động)
    raise SystemExit

print("[BOOT] Normal boot -> main.py will run")
print("[BOOT] Complete")
