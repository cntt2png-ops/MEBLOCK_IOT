import servo
import time
while True:
    servo.servo_270(pin=0, angle=90)
    time.sleep(1)   
    servo.servo_270(pin=0, angle=270)
    time.sleep(1)   
    servo.servo_270(pin=0, angle=0)
    time.sleep(1)