# `mqtt` — MQTT Helper

Chức năng chính của `mqtt`.

## Function

```sig
connect_wifi(ssid, password) -> ip
```
Kết nối Board MEBLOCK vào mạng Wifi của bạn. Trong đó:

    ssid là tên mạng Wifi.

    password là mật khẩu mạng WiFi.

```sig
connect_broker(server, port=1883, client_id=None)
```
Kết nối broker MQTT:

    server là  hostname hoặc IP broker (bắt buộc).

    port là cổng port broker, mặc định  = 1883.

    username tên user nếu broker cần xác thực.

    password là mặt khẩu nếu broker cần xác thực.

    client_id là ID client, nếu = None thì tự sinh.

    use_tls: True/False để bật SSL/TLS (nếu firmware và umqtt hỗ trợ)

    ssl_params tham số SSL (passed to client). 

```sig
on_receive_message(topic, callback)
```
Đăng ký callback xử lý message tới topic - `callback(payload, topic)`(hỗ trợ wildcard).

    topic: topic muốn subscribe (có thể chứa wildcard broker-side).

    callback: hàm callback nhận callback

        payload: bytes, topic: str
        

```sig
publish(topic, payload)
```
Gửi message tới topic.

    topic: topic đích.

    payload: nội dung truyền.


```sig
check_message()
```
Nhận tin **non‑block** và tự reconnect nếu socket lỗi.

    Kích hoạt **callback** nếu có message.


```sig
disconnect()
```
Ngắt kết nối

    Nên gọi trước khi tắt module hay đổi cấu hình.
    

## Sample Code

```python
import time, mqtt

mqtt.connect_wifi("SSID", "PASS")
mqtt.connect_broker("broker.hivemq.com", 1883)

def on_msg(payload, topic):
    print("RX:", topic, payload)

mqtt.on_receive_message("demo/esp32/#", on_msg)

while True:
    mqtt.check_message()
    mqtt.publish("demo/esp32/ping", "1")
    time.sleep(1)
```
