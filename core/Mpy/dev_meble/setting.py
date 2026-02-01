# setting.py
from utility import get_unique_suffix

DEVICE_NAME_PREFIX = "MEBLOCK-"
_SUFFIX_FILE = "device_name.txt"
_MAX_NAME_LEN = 20  # tổng độ dài name (BLE Adv nên <= 20 ký tự)


def _default_suffix():
    suffix = get_unique_suffix(6) or "TOPKID"
    return suffix


def _is_valid_char(ch):
    """
    Kiểm tra ký tự hợp lệ: A-Z, a-z, 0-9, '-', '_'
    (không dùng isalnum vì MicroPython build này không có)
    """
    if not ch:
        return False
    c = ord(ch)
    # 0-9
    if 48 <= c <= 57:
        return True
    # A-Z
    if 65 <= c <= 90:
        return True
    # a-z
    if 97 <= c <= 122:
        return True
    # thêm '-', '_'
    if ch == "-" or ch == "_":
        return True
    return False


def _sanitize_suffix(suffix):
    # Ép chắc chắn về string
    if not isinstance(suffix, str):
        suffix = str(suffix)

    suffix = suffix.strip().replace(" ", "-")

    allowed = []
    for ch in suffix:
        if _is_valid_char(ch):
            # chuyển thành UPPER cho đồng bộ
            allowed.append(ch.upper())

    suffix = "".join(allowed)
    if not suffix:
        suffix = _default_suffix()

    # Giới hạn độ dài: PREFIX + SUFFIX <= _MAX_NAME_LEN
    max_suffix_len = _MAX_NAME_LEN - len(DEVICE_NAME_PREFIX)
    if max_suffix_len < 1:
        max_suffix_len = 1

    if len(suffix) > max_suffix_len:
        suffix = suffix[:max_suffix_len]

    return suffix


def build_device_name(suffix):
    suffix = _sanitize_suffix(suffix)
    return DEVICE_NAME_PREFIX + suffix


def load_device_name():
    try:
        with open(_SUFFIX_FILE, "r") as f:
            suffix = f.read().strip()
    except Exception:
        suffix = _default_suffix()
        try:
            with open(_SUFFIX_FILE, "w") as f:
                f.write(suffix)
        except:
            pass
    return build_device_name(suffix)


def set_device_suffix(new_suffix):
    suffix = _sanitize_suffix(new_suffix)
    try:
        with open(_SUFFIX_FILE, "w") as f:
            f.write(suffix)
    except:
        pass
    return build_device_name(suffix)


def get_current_suffix():
    try:
        with open(_SUFFIX_FILE, "r") as f:
            return f.read().strip()
    except:
        return _default_suffix()
