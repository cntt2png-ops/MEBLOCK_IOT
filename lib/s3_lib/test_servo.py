import servo
import time

servo.servo_180(pin=19, angle= 90)
time.sleep(1)
servo.servo_180(pin=19, angle= 180)