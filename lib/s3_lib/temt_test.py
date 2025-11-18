from temt6000 import TEMT6000
import time

t = TEMT6000()   # default GPIO1

while True:
    print(
        "  pct =", t.percent_brightness(),
        "  lux~ =", t.lux()
    )
    time.sleep(0.5)
