# =======================
# MEBLOCK MQTT (Micropython) - v3 (M-channel + Dashboard password)
# =======================
# Topic chuẩn:
#   Command: meblock/{username}/{channel}/command
#   Data   : meblock/{username}/{channel}/data
#
# Logic đã thống nhất:
#   - Channel mặc định dạng M1, M2... (map legacy: "V1" -> "M1", "1" -> "M1")
#   - connect_dashboard(username, password=None)
#       + subscribe meblock/{username}/+/command (nghe mọi channel)
#       + nếu password được set:
#           * /command: chỉ nhận khi doc["password"] == password
#           * /data: tự gửi kèm {"password": password}
#
# Blockly blocks tối giản:
#   - connect_wifi(ssid, password)
#   - connect_broker(server, port, user, password, client_id=None)
#       -> CHỈ connect, KHÔNG subscribe
#   - connect_meblock(port=1883)
#   - connect_dashboard(username, password=None)
#   - on_receive_message(channel, action, callback)
#       -> callback(value, action, channel, username)
#   - check_message(auto_reconnect=True)  (gọi trong loop)
#   - send_value(channel, value)
#   - send_sensor_data(channel, temperature, humidity, battery, timestamp=None)

import time
import network
import machine
import ujson
import ubinascii
from umqtt.simple import MQTTClient
import gc, time, machine
_client = None

_cfg = {
    "server": None,
    "port": 1883,
    "user": None,
    "password": None,
    "client_id": None,
    "keepalive": 60,
}

_dashboard = {
    "username": None,
    "password": None,    # NEW: password cho dashboard/device
    "sub_topic": None,   # meblock/{username}/+/command
}

# Channel policy (NEW)
_CHAN_PREFIX = "M"          # dùng M1/M2...
_MAP_LEGACY_V = True        # map V1 -> M1

# Callback registry
# Exact match: (channel, action, value_key) -> cb
# - No wildcard for on_receive_message()
# - value_key is normalized string (e.g. "UP", "ON", "50", "#ff0000")
_cb_exact_val = {}   # {(ch, action, vkey): cb}

# Channel+Action match: (channel, action) -> cb
_cb_channel_action = {}  # {(ch, action): cb}

# Optional fallback APIs (keep for backward compatibility)
_cb_action = {}      # {action: cb}  (any channel/value)
_cb_any = None       # cb (receive any command)

# Last received value storage (for read_action_value)
# Key: (channel, action) -> last_value (soft-normalized)
_last_action_value = {}  # {(ch, action): value}

def _soft_int_value(v):
    """If v looks like a number => return int. Else return v as-is."""
    if v is None:
        return None
    # bool is subclass of int -> keep bool as-is
    if isinstance(v, bool):
        return v
    if isinstance(v, int):
        return v
    if isinstance(v, float):
        # soft: round then int
        try:
            return int(round(v))
        except Exception:
            return v
    if isinstance(v, str):
        s = v.strip()
        if not s:
            return v
        # integer?
        try:
            if (s[0] in "+-" and s[1:].isdigit()) or s.isdigit():
                return int(s)
        except Exception:
            pass
        # float?
        try:
            # accept "12.3", "-0.7"
            fv = float(s)
            return int(round(fv))
        except Exception:
            return v
    return v

def read_action_value(channel, action, default=None):
    """
    Read last received VALUE from a given (channel, action) from Dashboard commands.
    - channel: "M1", 1, "V1"... (will be normalized to Mx)
    - action : e.g. "DPAD_PRESS", "SLIDER_CHANGE", ...
    Returns:
      - last value
      - if value is numeric (or numeric string) => returns int
      - if never received => returns default (None if not provided)
    """
    ch = _normalize_channel(channel)
    act = _normalize_action(action)
    if act is None:
        return default
    return _last_action_value.get((ch, act), default)




MEBLOCK_SERVER = "103.173.226.204"
MEBLOCK_USER = "admin"
MEBLOCK_PASS = "Meblock@2026"

# ---------- Helpers ----------
def set_channel_prefix(prefix="M", map_legacy_v=True):
    """(Optional) thay đổi prefix channel. Mặc định là 'M'."""
    global _CHAN_PREFIX, _MAP_LEGACY_V
    _CHAN_PREFIX = (str(prefix or "M").strip() or "M")
    _MAP_LEGACY_V = bool(map_legacy_v)

def _normalize_channel(ch):
    """
    Normalize channel:
      - 1 / "1"   -> "M1"
      - "M1"/"m1" -> "M1"
      - "V1"/"v1" -> "M1" (nếu _MAP_LEGACY_V=True)
    """
    if ch is None:
        return "{}1".format(_CHAN_PREFIX)

    if isinstance(ch, int):
        return "{}{}".format(_CHAN_PREFIX, ch)

    s = str(ch).strip()
    if not s:
        return "{}1".format(_CHAN_PREFIX)

    if s.isdigit():
        return "{}{}".format(_CHAN_PREFIX, s)

    if len(s) >= 2 and s[1:].isdigit():
        h = s[0].upper()
        rest = s[1:]
        if h == _CHAN_PREFIX.upper():
            return "{}{}".format(_CHAN_PREFIX, rest)
        if _MAP_LEGACY_V and h == "V":
            return "{}{}".format(_CHAN_PREFIX, rest)

    return s


def _normalize_action(a):
    """Normalize action string to UPPERCASE."""
    if a is None:
        return None
    try:
        s = str(a).strip()
    except Exception:
        return None
    return s.upper() if s else None


def _normalize_value_key(v):
    """Normalize VALUE to a comparable key (string)."""
    if v is None:
        return None

    # bool is subclass of int, handle first
    if isinstance(v, bool):
        return "TRUE" if v else "FALSE"

    if isinstance(v, int):
        return str(v)

    if isinstance(v, float):
        try:
            if int(v) == v:
                return str(int(v))
        except Exception:
            pass
        return str(v)

    if isinstance(v, str):
        s = v.strip()
        if not s:
            return None
        # hex color keep lowercase for stability
        if s.startswith("#"):
            return s.lower()
        return s.upper()

    # dict/list: stringify
    try:
        return ujson.dumps(v)
    except Exception:
        try:
            return str(v)
        except Exception:
            return None


def _make_client_id(prefix="ESP32"):
    try:
        uid = ubinascii.hexlify(machine.unique_id()).decode()
    except Exception:
        uid = "0000"
    return "{}-{}".format(prefix, uid)

def _safe_decode(b):
    try:
        return b.decode()
    except Exception:
        return str(b)

def _parse_topic(topic):
    # meblock/{username}/{channel}/command
    parts = topic.split("/")
    if len(parts) >= 4 and parts[0] == "meblock":
        return parts[1], parts[2], parts[3]
    return None, None, None

def _extract_value(doc):
    # Ưu tiên dạng: {"value": ...}
    if isinstance(doc, dict) and ("value" in doc):
        return doc.get("value")
    # Chuẩn dạng: {"params":{"value":...}}
    params = (doc.get("params") or {}) if isinstance(doc, dict) else {}
    if isinstance(params, dict) and ("value" in params):
        return params.get("value")
    return None

def _call_cb(cb, value, action, channel, username, params, raw, topic):
    # Ưu tiên signature 4 args
    try:
        cb(value, action, channel, username)
        return
    except TypeError:
        pass
    # 5 args (thêm params)
    try:
        cb(value, action, channel, username, params)
        return
    except TypeError:
        pass
    # Debug: 7 args
    cb(value, action, channel, username, params, raw, topic)

def _password_ok(doc):
    pw = _dashboard.get("password")
    if not pw:
        return True
    if not isinstance(doc, dict):
        return False
    recv = doc.get("password")
    if recv is None:
        return False
    return str(recv) == str(pw)


# ---------- MQTT raw callback ----------
def _mqtt_callback(topic_b, payload_b):
    topic = _safe_decode(topic_b)
    raw = _safe_decode(payload_b)

    u_topic, ch_topic, kind = _parse_topic(topic)
    if kind != "command":
        return

    # Lọc theo dashboard đã chọn
    if _dashboard["username"] and u_topic != _dashboard["username"]:
        return

    # Parse JSON
    try:
        doc = ujson.loads(raw)
        if not isinstance(doc, dict):
            doc = {"value": doc}
    except Exception:
        doc = {"raw": raw}

    # Password check (NEW)
    if not _password_ok(doc):
        # giống Arduino: sai password -> bỏ qua
        print("=> ERROR: WRONG PASSWORD!")
        return

    action = doc.get("action") if isinstance(doc, dict) else None

    # Channel lấy từ JSON (chanel/channel) nếu có, fallback topic
    ch_doc = None
    if isinstance(doc, dict):
        ch_doc = doc.get("chanel") or doc.get("channel")
    channel = _normalize_channel(ch_doc or ch_topic)

    # Username lấy từ JSON nếu có, fallback topic
    u_doc = doc.get("username") if isinstance(doc, dict) else None
    username = u_doc or u_topic

    value = _extract_value(doc)
    params = doc.get("params") if isinstance(doc, dict) else None
    if params is None:
        params = {}

    # Dispatch (exact channel + action + value)
    act = _normalize_action(action)
    vkey = _normalize_value_key(value)

    if (act is None) or (vkey is None):
        return

    # store last value for read_action_value
    _last_action_value[(channel, act)] = _soft_int_value(value)

    cb = _cb_exact_val.get((channel, act, vkey))
    if cb:
        _call_cb(cb, value, act, channel, username, params, doc, topic)
        return

    # Channel+Action match
    cb = _cb_channel_action.get((channel, act))
    if cb:
        _call_cb(cb, value, act, channel, username, params, doc, topic)
        return

    # Fallback APIs (optional)
    cb = _cb_action.get(act)
    if cb:
        _call_cb(cb, value, act, channel, username, params, doc, topic)
        return

    if _cb_any:
        _call_cb(_cb_any, value, act, channel, username, params, doc, topic)
        return


# =======================
# Public API
# =======================
def clear_callbacks():
    """Clear all registered callbacks (useful in raw REPL)."""
    global _cb_exact_val, _cb_channel_action, _cb_action, _cb_any
    _cb_exact_val = {}
    _cb_channel_action = {}
    _cb_action = {}
    _cb_any = None

def connect_wifi(ssid, password, timeout_s=20, force_reinit=False):
    import gc
    global _wlan_sta, _wlan_ap
    gc.collect()
    # Tạo singleton để tránh tạo lại object WLAN nhiều lần
    try:
        _wlan_sta
    except NameError:
        _wlan_sta = network.WLAN(network.STA_IF)

    try:
        _wlan_ap
    except NameError:
        _wlan_ap = network.WLAN(network.AP_IF)

    # Tắt AP để tiết kiệm RAM
    try:
        _wlan_ap.active(False)
    except Exception:
        pass

    wlan = _wlan_sta

    # Thu gom rác trước khi bật WiFi
    gc.collect()

    # Nếu muốn làm sạch trạng thái WiFi cũ
    if force_reinit:
        try:
            wlan.disconnect()
        except Exception:
            pass
        try:
            wlan.active(False)
        except Exception:
            pass
        time.sleep_ms(300)
        gc.collect()

    # Bật STA (có retry nếu báo Out of Memory)
    try:
        wlan.active(True)
    except OSError as e:
        # thử “tắt/bật” lại + gc
        gc.collect()
        try:
            wlan.active(False)
            time.sleep_ms(300)
            wlan.active(True)
        except Exception as e2:
            print("WiFi FAIL (active):", e, e2)
            return False

    if wlan.isconnected():
        return True

    print("Dang ket noi WiFi {}...".format(ssid), end="")

    # Connect (có retry nếu Out of Memory)
    try:
        wlan.connect(ssid, password)
    except OSError as e:
        gc.collect()
        try:
            wlan.active(False)
            time.sleep_ms(300)
            wlan.active(True)
            wlan.connect(ssid, password)
        except Exception as e2:
            print("\nWiFi connect FAIL:", e, e2)
            return False

    t0 = time.time()
    while (not wlan.isconnected()) and (time.time() - t0 < timeout_s):
        print(".", end="")
        time.sleep(1)

    print()
    if wlan.isconnected():
        print("WiFi OK! IP:", wlan.ifconfig()[0])
        return True

    print("WiFi FAIL!")
    return False



def connect_broker(server, port=1883, user=None, password=None, client_id=None, keepalive=60):
    """
    CHỈ connect broker, KHÔNG subscribe.
    """
    global _client, _cfg

    _cfg["server"] = server
    _cfg["port"] = int(port)
    _cfg["user"] = user
    _cfg["password"] = password
    _cfg["keepalive"] = int(keepalive)
    _cfg["client_id"] = client_id or _make_client_id("MEBLOCK")

    try:
        _client = MQTTClient(
            _cfg["client_id"],
            _cfg["server"],
            port=_cfg["port"],
            user=_cfg["user"],
            password=_cfg["password"],
            keepalive=_cfg["keepalive"],
        )
        _client.set_callback(_mqtt_callback)
        _client.connect()
        print("MQTT OK! connected to", server)
        return True
    except Exception as e:
        print("MQTT connect FAIL:", e)
        _client = None
        return False


def connect_meblock(port=1883):
    return connect_broker(MEBLOCK_SERVER, port, MEBLOCK_USER, MEBLOCK_PASS)


def connect_dashboard(username, password=None):
    """
    Subscribe wildcard:
      meblock/{username}/+/command
    NEW:
      - có thể truyền password để lọc command + tự kèm password khi gửi data
    """
    if not _client:
        return False

    _dashboard["username"] = str(username)
    _dashboard["password"] = None if password is None else str(password)
    _dashboard["sub_topic"] = "meblock/{}/+/command".format(_dashboard["username"])

    try:
        _client.subscribe(_dashboard["sub_topic"])
        print("Subscribed:", _dashboard["sub_topic"])
        return True
    except Exception as e:
        print("Subscribe FAIL:", e)
        return False



def on_receive_message(channel, action, callback_or_value=None, callback=None):
    """
    Register callback by (channel, action) OR (channel, action, value).

    New (recommended):
        on_receive_message("M1", "SLIDER_CHANGE", cb)

    Backward-compatible (exact value match):
        on_receive_message("M1", "DPAD_PRESS", "UP", cb)

    Callback signature:
        cb(value, action, channel, username)
    """
    # Support both call styles:
    # - 3 args: (channel, action, cb)
    # - 4 args: (channel, action, value, cb)
    if callback is None and callable(callback_or_value):
        value = None
        callback = callback_or_value
    else:
        value = callback_or_value

    if callback is None or (not callable(callback)):
        return

    ch = _normalize_channel(channel)
    act = _normalize_action(action)
    if act is None:
        return

    # If value is None => register by (channel, action)
    if value is None:
        _cb_channel_action[(ch, act)] = callback
        return

    # Else exact match by value (legacy)
    vkey = _normalize_value_key(value)
    if vkey is None:
        return
    _cb_exact_val[(ch, act, vkey)] = callback

def on_receive_action(action, callback):
    """Register callback by action (any channel/value)."""
    if callback is None or (not callable(callback)):
        return
    act = _normalize_action(action)
    if act is None:
        return
    _cb_action[act] = callback


def check_message(auto_reconnect=True):
    """
    Gọi trong loop.
    """
    global _client
    if not _client:
        return False

    try:
        _client.check_msg()
        return True
    except Exception as e:
        print("MQTT check_msg error:", e)
        if auto_reconnect:
            return reconnect()
        return False


def reconnect(retry=3, delay_s=2):
    """Reconnect broker + resub dashboard wildcard."""
    global _client

    for _ in range(int(retry or 1)):
        try:
            try:
                if _client:
                    _client.disconnect()
            except Exception:
                pass

            _client = MQTTClient(
                _cfg["client_id"],
                _cfg["server"],
                port=_cfg["port"],
                user=_cfg["user"],
                password=_cfg["password"],
                keepalive=_cfg["keepalive"],
            )
            _client.set_callback(_mqtt_callback)
            _client.connect()

            # resub dashboard
            if _dashboard["sub_topic"]:
                _client.subscribe(_dashboard["sub_topic"])

            print("MQTT reconnected OK")
            return True
        except Exception as e:
            print("MQTT reconnect FAIL:", e)
            time.sleep(delay_s)

    _client = None
    return False


def _publish(topic, data_dict, retry_once=True, retained=False):
    global _client
    if not _client:
        return False

    try:
        _client.publish(topic, ujson.dumps(data_dict), retained)
        return True
    except Exception as e:
        print("Publish FAIL:", e)
        if retry_once and reconnect():
            try:
                _client.publish(topic, ujson.dumps(data_dict), retained)
                return True
            except Exception as e2:
                print("Publish retry FAIL:", e2)
        return False


def send_value(channel, value, include_channel_field=False, retained=False, **kwargs):
    """
    ESP -> Frontend (topic /data).
    Payload chuẩn theo bạn:
      {"value": ..., "password": "<dashboard_password>"}  (nếu password đã set)
    """
    if not _dashboard["username"]:
        raise RuntimeError("Chua chon dashboard. Goi connect_dashboard(username, password) truoc.")

    ch = _normalize_channel(channel)
    topic = "meblock/{}/{}/data".format(_dashboard["username"], ch)

    payload = {"value": value}
    # accept alias keywords: retain/retained
    if "retain" in kwargs:
        retained = bool(kwargs.get("retain"))
    if "retained" in kwargs:
        retained = bool(kwargs.get("retained"))

    # NEW: kèm password nếu có
    if _dashboard.get("password"):
        payload["password"] = _dashboard["password"]

    if include_channel_field:
        payload["channel"] = ch

    return _publish(topic, payload, retained=retained)

def send_string(channel, text, retained=False, **kwargs):
    """Gửi chuỗi (text) lên /data. Hỗ trợ retained/retain keyword."""
    # accept alias keywords
    if "retain" in kwargs:
        retained = bool(kwargs.get("retain"))
    if "retained" in kwargs:
        retained = bool(kwargs.get("retained"))
    try:
        s = text if isinstance(text, str) else str(text)
    except Exception:
        s = ""
    return send_value(channel, s, retained=retained)



# =======================
# JOYSTICK helpers (generic)
# direction = substring before '-'
# distance  = substring after  '-'
# examples: "UP-94", "UR-10", "J7-94", "LEFT-50"
# =======================

def _split_dir_dist(v):
    """
    Split value like 'DIR-94' -> ('DIR', 94, True).
    DIR can be any string (1..n chars).
    Distance tries int -> float, else fail.
    """
    if v is None:
        return "", None, False
    if not isinstance(v, str):
        try:
            v = str(v)
        except Exception:
            return "", None, False

    s = v.strip()
    if not s or "-" not in s:
        return "", None, False

    dir_part, dist_part = s.split("-", 1)
    dir_part = dir_part.strip()
    dist_part = dist_part.strip()

    if not dir_part:
        return "", None, False

    # parse number (int first, then float)
    try:
        if dist_part.isdigit() or (dist_part.startswith("-") and dist_part[1:].isdigit()):
            dist_num = int(dist_part)
        else:
            dist_num = float(dist_part)  # allows "94.5"
        return dir_part, dist_num, True
    except Exception:
        return dir_part, None, False


def joystick_dir(value, default=""):
    """Return direction string before '-'."""
    d, dist, ok = _split_dir_dist(value)
    return d if ok else default


def joystick_dist(value, default=-1):
    """Return distance number after '-' (int/float)."""
    d, dist, ok = _split_dir_dist(value)
    return dist if ok and dist is not None else default


def read_joystick_dir(channel, default=""):
    v = read_action_value(channel, "JOYSTICK_MOVE", None)
    return joystick_dir(v, default)


def read_joystick_dist(channel, default=-1):
    v = read_action_value(channel, "JOYSTICK_MOVE", None)
    return joystick_dist(v, default)

