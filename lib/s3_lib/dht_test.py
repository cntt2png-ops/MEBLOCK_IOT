import time
from aiot_dht import DHT11
from rgbled import RGBLed, RED, GREEN, BLUE
from time import sleep

led = RGBLed()
d1 = DHT11(pin=14)
led.color(RED)

while True:
    d1.read()
    print(d1.temperature(), d1.humidity())
    
    time.sleep(1)
