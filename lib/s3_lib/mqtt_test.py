import time
import machine
import ubinascii
import ujson

from s3_mqtt import connect_wifi, connect_broker, check_message, send_packet

# ====== CONFIG ======
WIFI_SSID = "TP-Link_B0EF"
WIFI_PASS = "34543795"

# Nếu dùng broker public (không cần user/pass)
MQTT_SERVER = "103.195.239.8"
MQTT_PORT   = 1883
MQTT_USER   = "admin"      # để rỗng nếu broker không cần
MQTT_PASS   = "Leto@n1989"      # để rỗng nếu broker không cần

# ====== TOPIC / CLIENT_ID ======
uid = ubinascii.hexlify(machine.unique_id())
client_id = b"meblock_" + uid

topic_base = b"meblock/test/" + uid
TOPIC_SUB  = topic_base + b"/cmd"   # gửi lệnh xuống
TOPIC_PUB  = topic_base + b"/tele"  # gửi dữ liệu lên

def on_mqtt_msg(topic, msg):
    print("<< MQTT RX:", topic, msg)
    # thử parse JSON
    try:
        data = ujson.loads(msg)
        print("   JSON =", data)
    except:
        pass

# ====== RUN ======
connect_wifi(WIFI_SSID, WIFI_PASS)

connect_broker(
    server=MQTT_SERVER,
    port=MQTT_PORT,
    user=MQTT_USER,
    password=MQTT_PASS,
    client_id=client_id,
    topic_sub=TOPIC_SUB,
    callback=on_mqtt_msg
)

# Gửi gói "hello" ban đầu
send_packet(TOPIC_PUB, {"hello": "meblock", "uid": uid.decode()})

last = time.ticks_ms()
i = 0

while True:
    check_message()

    # gửi định kỳ mỗi 2 giây
    if time.ticks_diff(time.ticks_ms(), last) > 2000:
        i += 1
        last = time.ticks_ms()
        send_packet(TOPIC_PUB, {"count": i, "ms": last})

    time.sleep_ms(20)
