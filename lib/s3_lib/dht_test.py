import time
from aiot_dht import DHT11
from rgbled import RGBLed, RED, GREEN, BLUE
from time import sleep

led = RGBLed()
d1 = DHT11(pin=4)
led.color(RED)

while True:
    d1.read()
    print(d1.temperature(), d1.humidity())
    if d1.temperature() > 30:
        for i in range(int(5)):
            led.color(GREEN)       # xanh lá
            sleep(0.5)
            led.color(RED)         # đỏ
            sleep(0.5)
    time.sleep(1)
