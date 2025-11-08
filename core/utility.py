# utility.py — tối thiểu, không bắt buộc nhưng để boot.py import cho sạch log
import time

def log(*args):
    # In kèm timestamp ms cho dễ đọc log
    ts = time.ticks_ms()
    print("[%.3fs]" % (ts / 1000), *args)

def get_ble_name(default="MEBLOCK-TOPKID"):
    # Nếu muốn dùng BLE_NAME trong setting.py cho ble.py sau này
    try:
        from setting import BLE_NAME
        return BLE_NAME or default
    except Exception:
        return default
