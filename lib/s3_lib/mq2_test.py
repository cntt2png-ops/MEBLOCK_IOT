from mq2 import MQ2
import time

mq2_sensor = MQ2()  
calibrated_ro = mq2_sensor.calibrate()
print("Calibrated Ro:", calibrated_ro)
while True:
    estimated_gas = mq2_sensor.estimate_ppm(gas="SMOKE")
    print("Estimated Smoke Concentration (PPM):", estimated_gas)
    level = mq2_sensor.get_level()[0]
    print("Smoke Level:", level)
    time.sleep(1)