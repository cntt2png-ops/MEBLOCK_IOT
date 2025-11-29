from oled import Oled
import time
oled = Oled(scl=9, sda=8, width=128, height=64)

oled.clear()
oled.text("Hello, S3!",64, 1, 1)
oled.show()
