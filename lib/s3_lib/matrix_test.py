from machine import Pin, I2C
from ht16k33matrix import HT16K33Matrix
import time

i2c = I2C(0, scl=Pin(9), sda=Pin(8), freq=400000)
matrix = HT16K33Matrix(i2c, i2c_address=0x70)

matrix.set_angle(2)       # xoay đúng như bạn nói
matrix.clear()
matrix.set_character(ord('A'), centre=True)
matrix.draw()
