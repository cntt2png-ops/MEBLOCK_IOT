# file: scan_i2c.py
from machine import SoftI2C, Pin

I2C_SCL = 10   # CHÂN I2C ĐANG DÙNG CHO MD4C
I2C_SDA = 11

i2c = SoftI2C(scl=Pin(I2C_SCL), sda=Pin(I2C_SDA), freq=100000)

print("Scanning I2C on SCL=%d SDA=%d ..." % (I2C_SCL, I2C_SDA))
devices = i2c.scan()
print("Found:", devices)

if not devices:
    print(">> KHÔNG THẤY THIẾT BỊ NÀO. Sai chân hoặc chưa cấp nguồn cho MD4C.")
else:
    for d in devices:
        print(" - 0x{:02X}".format(d))
