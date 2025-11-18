import time
from aiot_dht import DHT

d1= DHT()

while True:
    print(d1.temperature(), d1.humidity())
    time.sleep(2)