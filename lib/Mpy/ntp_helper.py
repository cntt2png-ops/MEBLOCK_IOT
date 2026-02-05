# ntp_helper.py (MicroPython ESP32)
# NTP + WiFi + Timezone (string: "UTC+7", "UTC+05:30", "GMT-8")
#
# API chính:
#   wifi_connect(ssid, pass, ...)
#   set_timezone("UTC+7")
#   update_time("UTC+7")   -> set TZ + sync NTP
#   sync_ntp()             -> chỉ sync NTP (RTC set UTC)
#   get_datetime(part="hour"/"minute"/"second"/"day"/"month"/"year", tz="UTC+7")
#
# Lưu ý: RTC được set theo UTC; khi đọc sẽ cộng offset TZ.

import time
import socket
import struct
from machine import RTC

# ===== Default NTP servers =====
_DEFAULT_HOSTS = (
    "time.google.com",
    "time.cloudflare.com",
    "asia.pool.ntp.org",
    "pool.ntp.org",
    "time.nist.gov",
)

# ===== State =====
_TZ_OFFSET = 7 * 3600  # default UTC+7
_synced = False
_last_host = None


# ================== TIMEZONE ==================
def _parse_tz_str(tz):
    """
    Accept:
      "UTC+7", "UTC+07", "UTC+07:00", "UTC+5:30", "GMT-8", "UTC", "GMT"
    Return: offset seconds (int).
    """
    if tz is None:
        raise ValueError("tz is None")

    s = str(tz).strip().upper().replace(" ", "")
    if s in ("UTC", "GMT", "UTC+0", "UTC+00", "UTC+00:00", "GMT+0", "GMT+00", "GMT+00:00"):
        return 0

    if s.startswith("UTC"):
        s = s[3:]
    elif s.startswith("GMT"):
        s = s[3:]

    if not s:
        return 0

    sign = 1
    if s[0] == "+":
        sign = 1
        s = s[1:]
    elif s[0] == "-":
        sign = -1
        s = s[1:]

    if not s:
        raise ValueError("Invalid timezone format")

    if ":" in s:
        hh_s, mm_s = s.split(":", 1)
        hh = int(hh_s)
        mm = int(mm_s)
    else:
        hh = int(s)
        mm = 0

    if not (0 <= hh <= 23):
        raise ValueError("TZ hours must be 0..23")
    if not (0 <= mm <= 59):
        raise ValueError("TZ minutes must be 0..59")

    return sign * (hh * 3600 + mm * 60)


def set_timezone(tz="UTC+7"):
    """Set timezone global. Example: set_timezone('UTC+7')"""
    global _TZ_OFFSET
    if isinstance(tz, str):
        _TZ_OFFSET = _parse_tz_str(tz)
    else:
        # allow: set_timezone(7) or set_timezone(-5)
        _TZ_OFFSET = int(tz) * 3600
    return _TZ_OFFSET


def get_timezone_offset():
    return _TZ_OFFSET


def get_timezone_str():
    off = _TZ_OFFSET
    sign = "+" if off >= 0 else "-"
    off = abs(off)
    hh = off // 3600
    mm = (off % 3600) // 60
    return "UTC%s%02d:%02d" % (sign, hh, mm)


# ================== WIFI (optional) ==================
def wifi_connect(ssid, password, timeout_ms=15000, static=None, verbose=False):
    """
    Kết nối WiFi.
      - static: (ip, subnet, gateway, dns) nếu muốn IP tĩnh
    """
    import network  # import khi dùng

    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)

    if static:
        wlan.ifconfig(static)

    if not wlan.isconnected():
        wlan.connect(ssid, password)
        t0 = time.ticks_ms()
        while not wlan.isconnected():
            if time.ticks_diff(time.ticks_ms(), t0) > timeout_ms:
                raise OSError("WiFi connect timeout")
            time.sleep_ms(200)

    if verbose:
        try:
            print("WiFi OK:", wlan.ifconfig())
        except Exception:
            pass

    return wlan


def wifi_is_connected():
    try:
        import network
        wlan = network.WLAN(network.STA_IF)
        return wlan.active() and wlan.isconnected()
    except Exception:
        return False


# ================== NTP ==================
def _ntp_delta():
    # ESP32 MicroPython thường epoch = 2000-01-01 -> delta 1900->2000
    base_year = time.gmtime(0)[0]
    return 3155673600 if base_year == 2000 else 2208988800  # 1900->2000 or 1900->1970


def sync_ntp(hosts=_DEFAULT_HOSTS, timeout=2, retries=2):
    """
    Sync RTC via NTP (RTC set UTC).
    Return True/False.
    """
    global _synced, _last_host
    _synced = False
    _last_host = None

    # if WiFi not connected, still try (user may use ethernet/other),
    # but most ESP32 cases require WiFi.
    NTP_DELTA = _ntp_delta()

    msg = bytearray(48)
    msg[0] = 0x1B

    last_err = None
    for host in hosts:
        for _ in range(retries):
            s = None
            try:
                addr = socket.getaddrinfo(host, 123)[0][-1]
                s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                s.settimeout(timeout)
                s.sendto(msg, addr)
                data = s.recv(48)
                s.close()

                # epoch seconds (MicroPython port epoch)
                t = struct.unpack("!I", data[40:44])[0] - NTP_DELTA
                tm = time.gmtime(t)

                # RTC datetime: (year, month, day, weekday, hour, minute, second, subsecond)
                RTC().datetime((tm[0], tm[1], tm[2], tm[6], tm[3], tm[4], tm[5], 0))

                _synced = True
                _last_host = host
                return True
            except Exception as e:
                last_err = e
                try:
                    if s:
                        s.close()
                except Exception:
                    pass

    # optional debug: print(last_err)
    return False


def update_time(tz="UTC+7", hosts=_DEFAULT_HOSTS, timeout=2, retries=2):
    """
    Cập nhật ngày giờ từ múi giờ mong muốn (vd: "UTC+7"):
      - set timezone global
      - sync NTP
    """
    set_timezone(tz)
    return sync_ntp(hosts=hosts, timeout=timeout, retries=retries)


def is_synced():
    return _synced


def last_host():
    return _last_host


# ================== READ DATETIME ==================
def _localtime(t=None, tz=None):
    """
    t: epoch UTC (time.time()) theo MicroPython port; nếu None => time.time()
    tz: None => dùng timezone global; hoặc "UTC+7"
    """
    if t is None:
        t = time.time()
    off = _TZ_OFFSET if tz is None else _parse_tz_str(tz)
    return time.localtime(t + off)


def get_datetime(part=None, t=None, tz=None):
    """
    1 hàm đọc thời gian theo phần:

    - get_datetime() -> dict: year, month, day, hour, minute, second
    - get_datetime("hour") -> trả về int
    - get_datetime(["day","month"]) -> dict subset

    part hợp lệ: year/month/day/hour/minute/second
    tz="UTC+7" để xem theo múi giờ bất kỳ (không đổi timezone global)
    """
    lt = _localtime(t=t, tz=tz)
    data = {
        "year": lt[0],
        "month": lt[1],
        "day": lt[2],
        "hour": lt[3],
        "minute": lt[4],
        "second": lt[5],
    }

    if part is None:
        return data

    if isinstance(part, str):
        key = part.strip().lower()
        if key not in data:
            raise ValueError("part must be one of: year/month/day/hour/minute/second")
        return data[key]

    if isinstance(part, (list, tuple)):
        out = {}
        for p in part:
            k = str(p).strip().lower()
            if k not in data:
                raise ValueError("part must be one of: year/month/day/hour/minute/second")
            out[k] = data[k]
        return out

    raise ValueError("part must be None, a string, or a list/tuple of strings")


def datetime_str(t=None, tz=None):
    lt = _localtime(t=t, tz=tz)
    return "%04d-%02d-%02d %02d:%02d:%02d" % (lt[0], lt[1], lt[2], lt[3], lt[4], lt[5])


def date_str(t=None, tz=None):
    lt = _localtime(t=t, tz=tz)
    return "%04d-%02d-%02d" % (lt[0], lt[1], lt[2])


def time_str(t=None, tz=None):
    lt = _localtime(t=t, tz=tz)
    return "%02d:%02d:%02d" % (lt[3], lt[4], lt[5])
