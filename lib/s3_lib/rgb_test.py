from onboard import get_rgb, RED, GREEN, BLUE
import time

led = get_rgb()          # hoặc led = OnboardRGB()

while True:
    led.red(150)
    time.sleep(0.3)
    led.green(150)
    time.sleep(0.3)
    led.blue(150)
    time.sleep(0.3)
    led.white(60)
    time.sleep(0.3)
    led.off()
    time.sleep(0.3)
