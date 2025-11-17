from rgbled import RGBLed, RED, GREEN, BLUE
from time import sleep
led = RGBLed()
while True: 
    # 3 màu cơ bản
    led.color(RED)         # đỏ
    sleep(1)
    led.color(GREEN)       # xanh lá
    sleep(1)
    led.color(BLUE)        # xanh dương
    sleep(1)

    # Trộn bằng OR
    led.color(RED | GREEN)             # vàng
    sleep(1)
    led.color(GREEN | BLUE)            # cyan
    sleep(1)
    led.color(RED | BLUE)              # magenta
    sleep(1)
    led.color(RED | GREEN | BLUE, 80)  # trắng 
    sleep(1)

    led.off()
    sleep(1)
    