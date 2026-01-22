# mqtt.py - MEBLOCK MQTT MicroPython (FORMAT CHUẨN THEO CODE C++ CỦA BẠN)
# Topics:
#   meblock/{username}/{channel}/data
#   meblock/{username}/{channel}/command
#
# Payload:
#   /data     : {"value": <...>, "password": "<device_password>"}
#   /command  : {"action": "...", "value": "...", "password": "<device_password>"}
#
# Channel: M1, M2... (nhập 1 -> M1). Có thể đổi prefix bằng set_channel_prefix().

import time
import network
import machine
import ubinascii
import ujson

try:
    from umqtt.simple import MQTTClient
except ImportError:
    MQTTClient = None

# Default broker (MEBlock)
MEBLOCK_SERVER = "103.195.239.8"
MEBLOCK_PORT = 1883
MEBLOCK_USER = "admin"
MEBLOCK_PASS = "Leto@n1989"

_client = None

_cfg = {
    "server": MEBLOCK_SERVER,
    "port": MEBLOCK_PORT,
    "user": MEBLOCK_USER,
    "password": MEBLOCK_PASS,
    "client_id": None,
    "keepalive": 60,
}

_identity = {
    "username": None,
    "channel": "M1",
    "device_password": None,
}

_chan = {
    "prefix": "M",
}

_on_command_any = None              # cb(action, value, channel, username, raw_dict)
_on_command_by_action = {}          # {"LED": cb, ...}


# ---------- Helpers ----------
def set_channel_prefix(prefix="M"):
    _chan["prefix"] = str(prefix or "M").strip() or "M"


def _normalize_channel(ch):
    """
    Accept:
      1, "1" -> "M1"
      "M1" -> "M1"
      "m2" -> "M2"
    """
    p = _chan["prefix"] or "M"
    if ch is None:
        return "{}1".format(p)

    if isinstance(ch, int):
        return "{}{}".format(p, ch)

    s = str(ch).strip()
    if s.isdigit():
        return "{}{}".format(p, s)

    # normalize "m1" -> "M1"
    if len(s) >= 2 and s[0].upper() == p.upper() and s[1:].isdigit():
        return "{}{}".format(p, s[1:])

    return s


def _make_client_id(prefix="MEBLOCK"):
    try:
        uid = ubinascii.hexlify(machine.unique_id()).decode()
    except Exception:
        uid = "0000"
    return "{}-{}".format(prefix, uid)


def _topic_data(username, channel):
    return "meblock/{}/{}/data".format(username, channel)


def _topic_command(username, channel):
    return "meblock/{}/{}/command".format(username, channel)


def _safe_decode(x):
    try:
        return x.decode()
    except Exception:
        return str(x)


def _publish(topic, payload_dict, retry_once=True):
    global _client
    if not _client:
        return False

    try:
        msg = ujson.dumps(payload_dict).encode()
        _client.publish(topic, msg)
        return True
    except Exception as e:
        print("Publish FAIL:", e)
        if retry_once and reconnect():
            try:
                msg = ujson.dumps(payload_dict).encode()
                _client.publish(topic, msg)
                return True
            except Exception as e2:
                print("Publish retry FAIL:", e2)
        return False


# ---------- WiFi ----------
def connect_wifi(ssid, password, timeout_s=20):
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)

    if wlan.isconnected():
        return True

    print("Dang ket noi WiFi {}...".format(ssid), end="")
    wlan.connect(ssid, password)

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


# ---------- MQTT connect ----------
def connect_broker(server=None, port=None, user=None, password=None, client_id=None, keepalive=60):
    """
    Connect MQTT broker only (no subscribe).
    """
    global _client

    if MQTTClient is None:
        raise RuntimeError("umqtt.simple not found in this firmware.")

    _cfg["server"] = server or _cfg["server"]
    _cfg["port"] = int(port or _cfg["port"])
    _cfg["user"] = user if user is not None else _cfg["user"]
    _cfg["password"] = password if password is not None else _cfg["password"]
    _cfg["keepalive"] = int(keepalive or 60)
    _cfg["client_id"] = client_id or _cfg["client_id"] or _make_client_id()

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
        print("MQTT OK! connected to", _cfg["server"])
        return True
    except Exception as e:
        print("MQTT connect FAIL:", e)
        _client = None
        return False


def connect_meblock(port=1883):
    return connect_broker(MEBLOCK_SERVER, port, MEBLOCK_USER, MEBLOCK_PASS)


def setup_identity(username, channel="M1", device_password=None):
    """
    Set username / channel / device_password (dùng để check command + gắn vào data)
    """
    _identity["username"] = str(username)
    _identity["channel"] = _normalize_channel(channel)
    _identity["device_password"] = None if device_password is None else str(device_password)


def subscribe_command(username=None, channel=None, device_password=None):
    """
    Subscribe đúng 1 topic command giống C++:
      meblock/{username}/{channel}/command
    """
    if not _client:
        return False

    if username is not None:
        _identity["username"] = str(username)
    if channel is not None:
        _identity["channel"] = _normalize_channel(channel)
    if device_password is not None:
        _identity["device_password"] = str(device_password)

    if not _identity["username"]:
        raise RuntimeError("Chua set username. Goi setup_identity(username, ...) truoc.")

    t = _topic_command(_identity["username"], _identity["channel"])
    try:
        _client.subscribe(t)
        print("Subscribed:", t)
        return True
    except Exception as e:
        print("Subscribe FAIL:", e)
        return False


def reconnect(retry=3, delay_s=2):
    """
    Reconnect + resubscribe command topic theo identity hiện tại.
    """
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

            # resub command
            if _identity["username"] and _identity["channel"]:
                _client.subscribe(_topic_command(_identity["username"], _identity["channel"]))

            print("MQTT reconnected OK")
            return True
        except Exception as e:
            print("MQTT reconnect FAIL:", e)
            time.sleep(delay_s)

    _client = None
    return False


def check_message(auto_reconnect=True):
    """
    Call trong loop để nhận command.
    """
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


# ---------- Command handling ----------
def on_command(callback):
    """
    callback(action, value, channel, username, raw_dict)
    """
    global _on_command_any
    _on_command_any = callback


def on_action(action, callback):
    """
    callback(action, value, channel, username, raw_dict)
    """
    _on_command_by_action[str(action)] = callback


def _password_ok(doc):
    pw = _identity["device_password"]
    if pw is None:
        return True
    recv = doc.get("password")
    if recv is None:
        return False
    return str(recv) == str(pw)


def _mqtt_callback(topic_b, payload_b):
    topic = _safe_decode(topic_b)
    raw = _safe_decode(payload_b)

    # Parse JSON
    try:
        doc = ujson.loads(raw)
        if not isinstance(doc, dict):
            doc = {"value": doc}
    except Exception:
        doc = {"raw": raw}

    # Password check giống C++
    if not _password_ok(doc):
        print("=> ERROR: WRONG PASSWORD!")
        return

    action = doc.get("action")
    value = doc.get("value")

    username = _identity["username"]
    channel = _identity["channel"]

    # Ưu tiên callback theo action
    cb = _on_command_by_action.get(str(action)) if action is not None else None
    if cb:
        cb(action, value, channel, username, doc)
        return

    if _on_command_any:
        _on_command_any(action, value, channel, username, doc)


# ---------- Publish data (FORMAT CHUẨN) ----------
def publish_value(value, username=None, channel=None, device_password=None):
    """
    Publish đúng format C++:
      {"value": value, "password": device_password}
    """
    if username is not None:
        _identity["username"] = str(username)
    if channel is not None:
        _identity["channel"] = _normalize_channel(channel)
    if device_password is not None:
        _identity["device_password"] = str(device_password)

    if not _identity["username"]:
        raise RuntimeError("Chua set username. Goi setup_identity(username, ...) truoc.")

    t = _topic_data(_identity["username"], _identity["channel"])
    payload = {"value": value}

    if _identity["device_password"] is not None:
        payload["password"] = _identity["device_password"]

    return _publish(t, payload)
