# setting.py
# Device naming helpers with cache + atomic save.

from utility import get_unique_suffix

DEVICE_NAME_PREFIX = "MEBLOCK-"
_SUFFIX_FILE = "device_name.txt"
_TEMP_FILE = "device_name.tmp"
_MAX_NAME_LEN = 20

_cached_suffix = None
_cached_name = None


def _default_suffix():
    suffix = get_unique_suffix(6) or "TOPKID"
    return suffix


def _is_valid_char(ch):
    if not ch:
        return False
    c = ord(ch)
    if 48 <= c <= 57:
        return True
    if 65 <= c <= 90:
        return True
    if 97 <= c <= 122:
        return True
    if ch == "-" or ch == "_":
        return True
    return False


def _sanitize_suffix(suffix):
    if not isinstance(suffix, str):
        suffix = str(suffix)

    suffix = suffix.strip().replace(" ", "-")

    allowed = []
    for ch in suffix:
        if _is_valid_char(ch):
            allowed.append(ch.upper())

    suffix = "".join(allowed)
    if not suffix:
        suffix = _default_suffix()

    max_suffix_len = _MAX_NAME_LEN - len(DEVICE_NAME_PREFIX)
    if max_suffix_len < 1:
        max_suffix_len = 1

    if len(suffix) > max_suffix_len:
        suffix = suffix[:max_suffix_len]

    return suffix


def build_device_name(suffix):
    suffix = _sanitize_suffix(suffix)
    return DEVICE_NAME_PREFIX + suffix


def _read_suffix_from_disk():
    try:
        with open(_SUFFIX_FILE, "r") as f:
            value = f.read().strip()
            if value:
                return _sanitize_suffix(value)
    except Exception:
        pass
    return _default_suffix()


def _write_suffix_to_disk(suffix):
    try:
        with open(_TEMP_FILE, "w") as f:
            f.write(suffix)
        try:
            import os
            try:
                os.remove(_SUFFIX_FILE)
            except Exception:
                pass
            os.rename(_TEMP_FILE, _SUFFIX_FILE)
        except Exception:
            with open(_SUFFIX_FILE, "w") as f:
                f.write(suffix)
        return True
    except Exception:
        return False


def _set_cache(suffix):
    global _cached_suffix, _cached_name
    _cached_suffix = suffix
    _cached_name = DEVICE_NAME_PREFIX + suffix
    return _cached_name


def load_device_name():
    global _cached_name
    if _cached_name is not None:
        return _cached_name

    suffix = _read_suffix_from_disk()
    _write_suffix_to_disk(suffix)
    return _set_cache(suffix)


def set_device_suffix(new_suffix):
    suffix = _sanitize_suffix(new_suffix)
    _write_suffix_to_disk(suffix)
    return _set_cache(suffix)


def get_current_suffix():
    global _cached_suffix
    if _cached_suffix is not None:
        return _cached_suffix

    suffix = _read_suffix_from_disk()
    _set_cache(suffix)
    return suffix
