# =======================
# MEBLOCK MQTT (Micropython) - v2
# =======================
# Chuẩn topic (theo doc):
#   Command: meblock/{username}/{channel}/command
#   Data   : meblock/{username}/{channel}/data
#
# Blockly blocks tối giản:
#   - connect_wifi(ssid, password)
#   - connect_broker(server, port, user, password, client_id=None)
#       -> CHỈ connect, KHÔNG subscribe
#   - connect_dashboard(username)
#       -> subscribe meblock/{username}/+/command (nghe mọi channel)
#   - on_receive_message(channel, action, callback)
#       -> callback(value, action, channel, username)
#   - check_message()  (gọi trong loop)
#   - send_value(channel, value)
#   - send_sensor_data(channel, temperature, humidity, battery, timestamp=None)

import time
import network
import machine
import ujson
import ubinascii
from umqtt.simple import MQTTClient

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
    "sub_topic": None,   # meblock/{username}/+/command
}

# Callback registry
# priority: (channel, action) -> (channel, "*") -> ("*", action) -> any
_cb_exact = {}       # {(ch, action): cb}
_cb_channel = {}     # {ch: cb}
_cb_action = {}      # {action: cb}
_cb_any = None       # cb


MEBLOCK_SERVER = "103.195.239.8"
MEBLOCK_USER = "admin"
MEBLOCK_PASS = "Leto@n1989"

# ---------- Helpers ----------
def _make_client_id(prefix="ESP32"):
    try:
        uid = ubinascii.hexlify(machine.unique_id()).decode()
    except:
        uid = "0000"
    return "{}-{}".format(prefix, uid)

def _safe_decode(b):
    try:
        return b.decode()
    except:
        return str(b)

def _parse_topic(topic):
    # meblock/{username}/{channel}/command
    parts = topic.split("/")
    if len(parts) >= 4 and parts[0] == "meblock":
        return parts[1], parts[2], parts[3]
    return None, None, None

def _extract_value(doc):
    # Ưu tiên dạng dạy học: {"value": ...}
    if isinstance(doc, dict) and ("value" in doc):
        return doc.get("value")
    # Chuẩn doc: {"params":{"value":...}}
    params = (doc.get("params") or {}) if isinstance(doc, dict) else {}
    if isinstance(params, dict) and ("value" in params):
        return params.get("value")
    return None

def _call_cb(cb, value, action, channel, username, params, raw, topic):
    # Ưu tiên signature “dễ Blockly”: 4 args
    try:
        cb(value, action, channel, username)
        return
    except TypeError:
        pass
    # Nâng cao: 5 args (thêm params)
    try:
        cb(value, action, channel, username, params)
        return
    except TypeError:
        pass
    # Debug: 7 args
    cb(value, action, channel, username, params, raw, topic)


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

    action = doc.get("action") if isinstance(doc, dict) else None

    # Channel lấy từ JSON (chanel/channel) nếu có, fallback topic
    ch_doc = None
    if isinstance(doc, dict):
        ch_doc = doc.get("chanel") or doc.get("channel")
    channel = ch_doc or ch_topic

    # Username lấy từ JSON nếu có, fallback topic
    u_doc = doc.get("username") if isinstance(doc, dict) else None
    username = u_doc or u_topic

    value = _extract_value(doc)
    params = doc.get("params") if isinstance(doc, dict) else None
    if params is None:
        params = {}

    # Dispatch
    cb = _cb_exact.get((channel, action))
    if cb:
        _call_cb(cb, value, action, channel, username, params, doc, topic)
        return

    cb = _cb_channel.get(channel)
    if cb:
        _call_cb(cb, value, action, channel, username, params, doc, topic)
        return

    cb = _cb_action.get(action)
    if cb:
        _call_cb(cb, value, action, channel, username, params, doc, topic)
        return

    if _cb_any:
        _call_cb(_cb_any, value, action, channel, username, params, doc, topic)


# =======================
# Public API
# =======================
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


def connect_broker(server, port=1883, user=None, password=None, client_id=None, keepalive=60):
    """
    CHỈ connect broker, KHÔNG subscribe.
    (khác bản cũ: connect_broker() từng subscribe + reset khi lỗi) :contentReference[oaicite:4]{index=4}
    """
    global _client, _cfg

    _cfg["server"] = server
    _cfg["port"] = port
    _cfg["user"] = user
    _cfg["password"] = password
    _cfg["keepalive"] = keepalive
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

def connect_dashboard(username):
    """
    Subscribe wildcard để bỏ block Subscribe Channel:
      meblock/{username}/+/command
    """
    if not _client:
        return False

    _dashboard["username"] = str(username)
    _dashboard["sub_topic"] = "meblock/{}/+/command".format(_dashboard["username"])

    try:
        _client.subscribe(_dashboard["sub_topic"])
        print("Subscribed:", _dashboard["sub_topic"])
        return True
    except Exception as e:
        print("Subscribe FAIL:", e)
        return False


def on_receive_message(channel, action=None, callback=None):
    """
    2 cách dùng:
      - on_receive_message("V1", "BUTTON_PRESS", cb)
      - on_receive_message("V1", cb)  (mọi action trên V1)
      - on_receive_message("*", "BUTTON_PRESS", cb) (mọi channel, lọc action)
    callback ưu tiên: cb(value, action, channel, username)
    """
    global _cb_any

    # on_receive_message("V1", cb)
    if callback is None and callable(action):
        _cb_channel[str(channel)] = action
        return

    # on_receive_message("*", "*", cb) -> any
    if str(channel) == "*" and (action == "*" or action is None):
        _cb_any = callback
        return

    # exact (channel, action)
    if callback:
        _cb_exact[(str(channel), str(action))] = callback


def on_receive_action(action, callback):
    """Lọc theo action cho mọi channel."""
    _cb_action[str(action)] = callback


def on_receive_any(callback):
    """Nhận mọi command."""
    global _cb_any
    _cb_any = callback


def check_message(auto_reconnect=True):
    """
    Gọi trong loop.
    Bản cũ reset chip khi mất kết nối :contentReference[oaicite:5]{index=5},
    bản mới sẽ cố reconnect nếu auto_reconnect=True.
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

    for _ in range(retry):
        try:
            try:
                if _client:
                    _client.disconnect()
            except:
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


def _publish(topic, data_dict, retry_once=True):
    global _client
    if not _client:
        return False

    try:
        _client.publish(topic, ujson.dumps(data_dict))
        return True
    except Exception as e:
        print("Publish FAIL:", e)
        if retry_once and reconnect():
            try:
                _client.publish(topic, ujson.dumps(data_dict))
                return True
            except Exception as e2:
                print("Publish retry FAIL:", e2)
        return False


def send_value(channel, value, include_channel_field=False):
    """
    ESP -> Frontend (topic /data). Payload tối giản: {"value": ...}
    """
    if not _dashboard["username"]:
        raise RuntimeError("Chua chon dashboard. Goi connect_dashboard(username) truoc.")

    ch = str(channel)
    topic = "meblock/{}/{}/data".format(_dashboard["username"], ch)

    payload = {"value": value}
    if include_channel_field:
        payload["channel"] = ch

    return _publish(topic, payload)


def send_sensor_data(channel, temperature=None, humidity=None, battery=None, timestamp=None, extra=None):
    """
    ESP -> Frontend (topic /data). Payload dạng sensor: {temperature, humidity, battery, timestamp, ...}
    """
    if not _dashboard["username"]:
        raise RuntimeError("Chua chon dashboard. Goi connect_dashboard(username) truoc.")

    ch = str(channel)
    topic = "meblock/{}/{}/data".format(_dashboard["username"], ch)

    payload = {}
    if temperature is not None:
        payload["temperature"] = temperature
    if humidity is not None:
        payload["humidity"] = humidity
    if battery is not None:
        payload["battery"] = battery
    if timestamp is not None:
        payload["timestamp"] = timestamp
    if isinstance(extra, dict):
        payload.update(extra)

    return _publish(topic, payload)
