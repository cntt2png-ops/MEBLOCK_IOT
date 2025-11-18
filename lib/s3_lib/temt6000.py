# temt6000.py
# Simple MicroPython driver for Keyestudio KS0098 (TEMT6000) light sensor
# Works on ESP32 / ESP32-S3

from machine import ADC, Pin
import time

# Default ADC pin for ESP32-S3 DevKitC-1 (change if you wire differently)
S3_TEMT6000_ADC = 1   # GPIO1


class TEMT6000:
    def __init__(self, pin=None, vref=3.3, samples=10):
        # If no pin is given, use S3 preset
        if pin is None:
            pin = S3_TEMT6000_ADC

        self.vref = float(vref)
        self.samples = int(samples)

        # 12-bit ADC on ESP32 -> max value 4095
        self.adc_max = 4095

        self.adc = ADC(Pin(pin))
        try:
            # 11 dB attenuation -> full range about 0-3.3 V
            self.adc.atten(ADC.ATTN_11DB)
            self.adc.width(ADC.WIDTH_12BIT)
        except:
            # Some ports may not support these methods
            pass

    def _read_raw_once(self):
        return self.adc.read()

    def read_raw(self):
        """Return averaged raw ADC value."""
        total = 0
        for _ in range(self.samples):
            total += self._read_raw_once()
            time.sleep_ms(2)
        return total // self.samples

    def read_voltage(self):
        """Return sensor voltage in Volts."""
        raw = self.read_raw()
        return raw * self.vref / self.adc_max

    def percent_brightness(self):
        """Return relative brightness in percent (0-100)."""
        raw = self.read_raw()
        pct = raw * 100.0 / self.adc_max
        return round(pct, 1)

    def lux(self):
        """
        Rough lux estimation.
        This is only for educational/demo purposes.
        """
        raw = self.read_raw()
        ratio = raw / self.adc_max
        lux = (ratio ** 1.4) * 1000.0
        return round(lux, 1)
