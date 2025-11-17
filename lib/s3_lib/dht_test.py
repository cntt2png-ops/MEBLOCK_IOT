import time
from aiot_dht import DHT20, DHT11

d1= DHT11(pin=4)

while True:
    print(d1.temperature(), d1.humidity())
    time.sleep(2)