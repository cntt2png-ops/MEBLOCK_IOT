from onboard import get_rgb, RED, GREEN, BLUE
import time

led = get_rgb()         

while True:
    led.red(150)
    time.sleep(0.5)
    print("Test\n")
    led.green(150)
    time.sleep(0.5)
    led.blue(150)
    time.sleep(0.5)
    led.white(60)
    time.sleep(0.5)
    led.off()
    time.sleep(2)
