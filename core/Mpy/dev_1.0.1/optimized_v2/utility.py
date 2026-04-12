# utility.py
# System helpers with lightweight caching where safe.

from machine import unique_id
import sys
import gc
import os

_cached_uid = None


def get_unique_id_bytes():
    global _cached_uid
    if _cached_uid is not None:
        return _cached_uid
    try:
        _cached_uid = unique_id()
    except Exception:
        _cached_uid = b""
    return _cached_uid


def get_unique_id_hex():
    uid = get_unique_id_bytes()
    if not uid:
        return ""
    return uid.hex().upper()


def get_unique_suffix(length=6):
    uid = get_unique_id_bytes()
    if not uid:
        return ""
    needed_bytes = max(1, (int(length) + 1) // 2)
    part = uid[-needed_bytes:]
    return part.hex().upper()[:length]


def get_firmware_version():
    impl = sys.implementation
    return {
        "name": impl.name,
        "version": ".".join(str(v) for v in impl.version),
        "mpy": getattr(impl, "mpy", None),
        "platform": sys.platform,
    }


def get_free_mem():
    gc.collect()
    return gc.mem_free()


def get_fs_info(path="/"):
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
    return {
        "uid_hex": get_unique_id_hex(),
        "firmware": get_firmware_version(),
        "free_mem": get_free_mem(),
        "fs": get_fs_info("/") or {},
    }
