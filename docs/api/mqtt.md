# `mqtt` — MQTT Helper

Các hàm tiện ích kết nối **Wi‑Fi** và **MQTT broker**.

## Function

`connect_wifi(ssid, password, timeout_s=15) -> ip`  
Kết nối Wi‑Fi, trả về IP nếu thành công.

`connect_broker(server, port=1883, username=None, password=None, *, client_id=None, keepalive=30, use_tls=False, ssl_params=None, clean_session=True, will_topic=None, will_msg=None, will_qos=0, will_retain=False)`  
Kết nối tới MQTT broker, cấu hình **Last Will** nếu cần.

`on_receive_message(topic, callback, *, qos=0)`  
Đăng ký hàm `callback(payload, topic)` cho `topic` (có hỗ trợ wildcard).

`check_message()`  
Non‑block, gọi các callback khi có tin đến; tự reconnect nếu socket lỗi.

`publish(topic, payload, *, qos=0, retain=False)`  
Gửi dữ liệu lên broker.

`disconnect()`  
Ngắt kết nối.

## Sample Code

```python
import time
import mqtt

mqtt.connect_wifi("SSID", "PASS")
mqtt.connect_broker("broker.hivemq.com", 1883, client_id="esp32-demo")

def on_msg(payload, topic):
    print("RX:", topic, payload)

mqtt.on_receive_message("demo/esp32/#", on_msg)

i = 0
while True:
    mqtt.check_message()
    mqtt.publish("demo/esp32/uptime", str(i))
    i += 1
    time.sleep(1)
```
