# API DOCUMENTATION:
#   connect_wifi(ssid, password)
#       -> Kết nối WiFi. Timeout 20s -> Reset chip.
#
#   connect_broker(server, port, user, password, client_id, topic_sub=None, callback=None)
#       -> Kết nối Broker, gán callback và subscribe. Lỗi -> Reset chip.
#
#   check_message()
#       -> Kiểm tra tin nhắn đến (đặt trong loop). Mất kết nối -> Reset chip.
#
#   send_packet(topic, data_dict)
#       -> Gửi dữ liệu dạng Dictionary (JSON).

import time
import network
import machine
import ujson
from umqtt.simple import MQTTClient

_client = None

def connect_wifi(ssid, password):
    wlan = network.WLAN(network.STA_IF)
    wlan.active(False)
    time.sleep(1)
    wlan.active(True)
    
    if not wlan.isconnected():
        print(f'Dang ket noi WiFi {ssid}...', end='')
        wlan.connect(ssid, password)
        timeout = 20
        while not wlan.isconnected() and timeout > 0:
            print('.', end='')
            time.sleep(1)
            timeout -= 1
            
    if wlan.isconnected():
        print('\nWiFi OK! IP:', wlan.ifconfig()[0])
    else:
        print('\nKet noi WiFi THAT BAI. Reset chip...')
        machine.reset()

def connect_broker(server, port, user, password, client_id, topic_sub=None, callback=None):
    global _client
    print(f"Dang ket noi Broker {server}...")
    
    try:
        _client = MQTTClient(client_id, server, port=port, user=user, password=password, keepalive=60)
        
        if callback:
            _client.set_callback(callback)
            
        _client.connect()
        print("=> Da ket noi MQTT thanh cong!")
        
        if topic_sub:
            _client.subscribe(topic_sub)
            print(f"=> Dang lang nghe tai: {topic_sub}")

        return _client
    except Exception as e:
        print(f"Loi ket noi MQTT: {e}")
        time.sleep(2)
        machine.reset()

def check_message():
    global _client
    if _client:
        try:
            _client.check_msg()
        except OSError:
            print("Mat ket noi MQTT. Reset...")
            machine.reset()

def send_packet(topic, data_dict):
    global _client
    if _client:
        try:
            msg_json = ujson.dumps(data_dict)
            _client.publish(topic, msg_json)
            print(f">> Da gui den {topic}: {msg_json}")
        except Exception as e:
            print(f"Loi khi gui tin: {e}")