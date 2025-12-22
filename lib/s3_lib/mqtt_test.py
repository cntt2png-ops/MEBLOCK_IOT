import time
import machine
import mqtt

# ====== CONFIG ======
SSID = "AIOT"
PASSWORD = "123456789"

MQTT_SERVER = "103.195.239.8"
MQTT_PORT = 1883
MQTT_USER = "admin"
MQTT_PASS = "Leto@n1989"

DASHBOARD_USERNAME = "abc123"

#Test

mqtt.connect_wifi(SSID, PASSWORD)
mqtt.connect_meblock(1883)
mqtt.connect_dashboard(DASHBOARD_USERNAME)
payload = {
  "temperature": 60.0,
  "humidity": 30.0,
  "battery": 95,
  "timestamp": "2025-12-20T10:00:00"
}

while True:
    mqtt.send_value("V1", payload)
    time.sleep(100)
