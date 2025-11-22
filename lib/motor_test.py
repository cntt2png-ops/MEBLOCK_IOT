from i2c_motors_driver import DCMotorV1, MD4C_REG_CH4, MD4C_REG_CH1, MD4C_REG_CH2, MD4C_REG_CH3
from onboard import get_rgb, RED, GREEN, BLUE
import time
# Ví dụ: S3 DevKitC-1, chọn chân bất kỳ hỗ trợ GPIO
dc = DCMotorV1(scl_pin=10, sda_pin=11)   # thay bằng chân S3 
led = get_rgb()  
led.red(150)
dc.setSpeed(MD4C_REG_CH1, 25)           # motor CH1 chạy forward 50%
time.sleep(2)
dc.setSpeed(MD4C_REG_CH2, 50)           # motor CH1 chạy forward 50%
time.sleep(2)
dc.setSpeed(MD4C_REG_CH3, 75)           # motor CH1 chạy forward 50%
time.sleep(2)
dc.setSpeed(MD4C_REG_CH4, 100)           # motor CH1 chạy forward 50%
time.sleep(5)

dc.setSpeed(MD4C_REG_CH3, 0)
dc.setSpeed(MD4C_REG_CH2, 0)
dc.setSpeed(MD4C_REG_CH1, 0)
led.off()



