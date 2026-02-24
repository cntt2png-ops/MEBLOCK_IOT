# utility.py
# Các hàm tiện ích lấy thông tin hệ thống, chip, bộ nhớ...

from machine import unique_id
import sys
import gc
import os


def get_unique_id_bytes():
    """
    Trả về unique_id dạng bytes (tùy board: 6/8/16 bytes).
    """
    try:
        return unique_id()
    except Exception:
        return b""


def get_unique_id_hex():
    """
    Trả về unique_id dạng hex string, ví dụ 'A1B2C3D4E5F6'.
    """
    uid = get_unique_id_bytes()
    if not uid:
        return ""
    return uid.hex().upper()


def get_unique_suffix(length=6):
    """
    Lấy suffix mặc định từ unique_id, độ dài 'length' ký tự hex.
    - Mặc định 6 ký tự -> dùng 3 byte cuối.
    """
    uid = get_unique_id_bytes()
    if not uid:
        return ""
    # Lấy đủ số byte cần thiết từ cuối chuỗi
    needed_bytes = max(1, length // 2)
    part = uid[-needed_bytes:]
    return part.hex().upper()[:length]


def get_firmware_version():
    """
    Trả về info firmware MicroPython:
    (name, version tuple, mpy ABI, platform)
    """
    impl = sys.implementation
    return {
        "name": impl.name,
        "version": ".".join(str(v) for v in impl.version),
        "mpy": getattr(impl, "mpy", None),
        "platform": sys.platform,
    }


def get_free_mem():
    """
    Trả về RAM trống (bytes).
    """
    gc.collect()
    return gc.mem_free()


def get_fs_info(path="/"):
    """
    Trả về thông tin filesystem trên 'path'.
    """
    try:
        stat = os.statvfs(path)
        block_size = stat[0]
        total_blocks = stat[2]
        free_blocks = stat[3]
        return {
            "block_size": block_size,
            "total_blocks": total_blocks,
            "free_blocks": free_blocks,
            "total_bytes": block_size * total_blocks,
            "free_bytes": block_size * free_blocks,
        }
    except Exception:
        return None


def get_system_summary():
    """
    Gói gọn thông tin hệ thống cơ bản thành dict.
    Có thể dùng cho lệnh debug qua Web Bluetooth.
    """
    return {
        "uid_hex": get_unique_id_hex(),
        "firmware": get_firmware_version(),
        "free_mem": get_free_mem(),
        "fs": get_fs_info("/") or {},
    }
