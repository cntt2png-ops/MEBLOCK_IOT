# mqtt.py — Thư viện MQTT MicroPython/ESP32
# API:
#   connect_wifi(ssid, password, timeout_s=15) -> ip
#   connect_broker(server, port=1883, username=None, password=None, *,
#                  client_id=None, keepalive=30, use_tls=False, ssl_params=None,
#                  clean_session=True, will_topic=None, will_msg=None,
#                  will_qos=0, will_retain=False)
#   publish(topic, payload, *, qos=0, retain=False)
#   on_receive_message(topic, callback, *, qos=0)
#   check_message()
#   disconnect()

import time
import network
import ubinascii
import machine

try:
    # MicroPython's built-in MQTT client
    from umqtt.simple import MQTTClient
except Exception as e:
    raise RuntimeError("umqtt.simple không có sẵn trong firmware: %r" % e)

# ---- Trạng thái toàn cục ----
_client = None
_connected = False
_handlers = {}   # topic(str) -> callback(payload: bytes, topic: str)
_subs = set()    # các topic đã subscribe
_conn_cfg = {}   # tham số kết nối để reconnect
_backoff_s = 1

# ---- Wi-Fi ----
def connect_wifi(ssid, password, timeout_s=15):
    sta = network.WLAN(network.STA_IF)
    if not sta.active():
        sta.active(True)
    if not sta.isconnected():
        sta.connect(ssid, password)
        t0 = time.ticks_ms()
        while not sta.isconnected():
            time.sleep_ms(200)
            if time.ticks_diff(time.ticks_ms(), t0) > timeout_s*1000:
                break
    if not sta.isconnected():
        raise OSError("Wi-Fi connect timeout")
    ip = sta.ifconfig()[0]
    print("[wifi] connected:", ip)
    return ip

# ---- Nội bộ: tạo client & kết nối ----
def _make_client(cfg):
    # sinh client_id nếu không truyền
    cid = cfg.get("client_id")
    if not cid:
        tail = ubinascii.hexlify(machine.unique_id()).decode()[-4:]
        cid = "esp32-%s" % tail

    c = MQTTClient(
        client_id=cid,
        server=cfg["server"],
        port=cfg.get("port", 1883),
        user=cfg.get("username", None),
        password=cfg.get("password", None),
        keepalive=cfg.get("keepalive", 30),
        ssl=cfg.get("use_tls", False),
        ssl_params=cfg.get("ssl_params", None),
    )

    # LWT (nếu có)
    wt = cfg.get("will_topic")
    wm = cfg.get("will_msg")
    if wt is not None and wm is not None:
        # umqtt.simple: set_last_will(topic, msg, retain=False, qos=0)
        c.set_last_will(wt, wm, retain=cfg.get("will_retain", False), qos=cfg.get("will_qos", 0))

    # callback chung -> phân phối theo _handlers
    def _on_message(topic_b, payload_b):
        try:
            t = topic_b.decode() if isinstance(topic_b, (bytes, bytearray)) else str(topic_b)
        except:
            t = str(topic_b)
        # Tìm callback khớp topic (ưu tiên match exact; wildcard do broker xử lý lúc subscribe)
        cb = _handlers.get(t, None)
        if cb:
            try:
                cb(payload_b, t)
            except Exception as e:
                # tránh văng vòng lặp
                print("[mqtt] handler error:", e)

        # Nếu sub bằng wildcard, broker đã match rồi nhưng key trong _handlers là wildcard.
        # Ta thử gọi các handler wildcard nếu có:
        for patt, h in _handlers.items():
            if patt.endswith("/#") or "/+" in patt or patt == "#":
                # không kiểm tra pattern phức tạp ở client; vì broker đã route đúng,
                # ta chỉ gọi khi handler đăng ký wildcard và không phải exact key.
                if h is not cb:
                    try:
                        h(payload_b, t)
                    except Exception as e:
                        print("[mqtt] wildcard handler error:", e)

    c.set_callback(_on_message)
    return c

def _connect_and_subscribe():
    global _client, _connected
    _client.connect(clean_session=_conn_cfg.get("clean_session", True))
    _connected = True
    # resubscribe các topic đã có handler
    for t in _handlers.keys():
        if t not in _subs:
            qos = 0
            _client.subscribe(t, qos=qos)
            _subs.add(t)
    print("[mqtt] connected & resubscribed:", len(_subs), "topics")

# ---- Public: connect_broker ----
def connect_broker(server, port=1883, username=None, password=None, *,
                   client_id=None, keepalive=30, use_tls=False, ssl_params=None,
                   clean_session=True, will_topic=None, will_msg=None, will_qos=0, will_retain=False):
    """
    Kết nối tới broker và ghi nhớ cấu hình để có thể reconnect.
    """
    global _client, _connected, _conn_cfg, _backoff_s
    _conn_cfg = dict(server=server, port=port, username=username, password=password,
                     client_id=client_id, keepalive=keepalive,
                     use_tls=use_tls, ssl_params=ssl_params,
                     clean_session=clean_session,
                     will_topic=will_topic, will_msg=will_msg,
                     will_qos=will_qos, will_retain=will_retain)
    _backoff_s = 1
    _client = _make_client(_conn_cfg)
    _connect_and_subscribe()
    print("[mqtt] broker:", server, port)

# ---- Public: publish ----
def publish(topic, payload, *, qos=0, retain=False):
    if not _connected:
        raise OSError("MQTT not connected")
    if isinstance(payload, str):
        payload = payload.encode()
    _client.publish(topic, payload, retain=retain, qos=qos)

# ---- Public: on_receive_message ----
def on_receive_message(topic, callback, *, qos=0):
    """
    Đăng ký callback cho topic (có thể wildcard). Nếu đang kết nối sẽ subscribe luôn.
    callback(payload: bytes, topic: str) -> None
    """
    _handlers[topic] = callback
    if _connected and topic not in _subs:
        _client.subscribe(topic, qos=qos)
        _subs.add(topic)

# ---- Public: check_message ----
def check_message(max_messages=1):
    """
    Bơm vòng nhận non-block. Mỗi lần gọi cố gắng lấy tối đa max_messages (mặc định 1).
    Nếu mất kết nối, sẽ tự reconnect (có backoff).
    """
    global _client, _connected, _backoff_s
    if not _client:
        raise RuntimeError("Broker not configured. Call connect_broker() first.")

    count = 0
    while count < max_messages:
        try:
            _client.check_msg()  # non-block; gọi callback nếu có
            count += 1
        except OSError as e:
            # Mất kết nối -> reconnect
            _connected = False
            print("[mqtt] socket error, reconnecting in", _backoff_s, "s:", e)
            time.sleep(_backoff_s)
            _backoff_s = min(_backoff_s * 2, 30)
            try:
                _client = _make_client(_conn_cfg)
                _connect_and_subscribe()
                _backoff_s = 1
                break
            except Exception as e2:
                print("[mqtt] reconnect failed:", e2)
                # sẽ thử lại ở lần check_message sau
                break
        except Exception as e:
            # Lỗi khác: log nhẹ
            print("[mqtt] check_msg error:", e)
            break

# ---- Public: disconnect ----
def disconnect():
    global _client, _connected
    if _client:
        try:
            _client.disconnect()
        except:
            pass
    _connected = False
