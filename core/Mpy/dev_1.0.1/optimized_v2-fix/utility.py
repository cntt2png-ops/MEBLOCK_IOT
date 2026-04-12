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


def build_info_text():
    summary = get_system_summary()
    fw = summary.get("firmware") or {}
    fs = summary.get("fs") or {}
    parts = [
        "uid=%s" % (summary.get("uid_hex") or ""),
        "fw=%s %s" % (fw.get("name") or "", fw.get("version") or ""),
        "mpy=%s" % (fw.get("mpy") if fw.get("mpy") is not None else ""),
        "platform=%s" % (fw.get("platform") or ""),
        "free_mem=%s" % (summary.get("free_mem") if summary.get("free_mem") is not None else ""),
        "fs_free=%s" % (fs.get("free_bytes") if fs.get("free_bytes") is not None else ""),
        "fs_total=%s" % (fs.get("total_bytes") if fs.get("total_bytes") is not None else ""),
    ]
    return "; ".join(parts)


def _write_raw(out, text):
    data = text.encode("utf-8") if isinstance(text, str) else text

    if out is None:
        try:
            print(text)
        except Exception:
            pass
        return data

    if callable(out):
        try:
            out(data)
            return data
        except TypeError:
            out(text)
            return data
        except Exception:
            return data

    for name in ("write", "send"):
        fn = getattr(out, name, None)
        if fn:
            try:
                fn(data)
                return data
            except TypeError:
                fn(text)
                return data
            except Exception:
                return data

    return data


def send_info(out=None, code="I_SYSINFO"):
    text = build_info_text()
    try:
        from onboard import send_inf
        return send_inf(code, text, out=out)
    except Exception:
        return _write_raw(out, text)
