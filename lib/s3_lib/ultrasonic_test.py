from ultrasonic import HCSR04
import time
hr = HCSR04()

while True:
    dist = hr.distance_cm()
    print("Khoảng cách: {:.2f} cm".format(dist))
    time.sleep(1)