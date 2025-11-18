from ultrasonic import HCSR04
import time
hr = HCSR04(trigger_pin=1, echo_pin=2)

while True:
    dist = hr.distance_cm()
    print("Khoảng cách: {:.2f} cm".format(dist))
    time.sleep(1)