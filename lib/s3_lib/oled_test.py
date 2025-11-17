from oled import Oled
import time
oled = Oled(width=128, height=64)

oled.clear()
oled.dev.text("Hello, S3!", 0, 0, 1)
oled.show()
i=0
while True:
    oled.clear()
    oled.dev.text("OLED Test " + str(i), 0, 16, 1)
    oled.show()
    i += 1
    time.sleep(1)