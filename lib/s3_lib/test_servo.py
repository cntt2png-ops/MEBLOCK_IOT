import servo
import time

servo.servo_360(pin=1, speed_percent=100)
time.sleep(1)
servo.servo_360(pin=1, speed_percent=-100)