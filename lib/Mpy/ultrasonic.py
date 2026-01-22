# ultrasonic.py – Especially for ESP32-S3 DevKitC-1 (preset)

import time, machine
from machine import Pin

try:
    const
except NameError:
    def const(x): return x

_MAX_DISTANCE_CM = const(200)

# DevKitC-1 preset - chosen pins for other pins in parameters 
S3_US_TRIG = 1
S3_US_ECHO = 2


class HCSR04:
    """
    Driver HC-SR04 cho ESP32-S3.

    Nếu không chọn trig/echo -> dùng S3_US_TRIG / S3_US_ECHO
    """

    def __init__(self,
                 trigger_pin=None,
                 echo_pin=None,
                 *,
                 echo_timeout_us=500*2*30,
                 trigger_pull=None,
                 echo_pull=None,
                 trigger_active_high=True):
        self.echo_timeout_us = int(echo_timeout_us)
        self.trigger_active_high = bool(trigger_active_high)

        self._ars = []
        self._ats = []

        trig_id = S3_US_TRIG if trigger_pin is None else int(trigger_pin)
        echo_id = S3_US_ECHO if echo_pin is None else int(echo_pin)

        self.trigger = Pin(trig_id, mode=Pin.OUT, pull=trigger_pull)
        self.echo    = Pin(echo_id, mode=Pin.IN,  pull=echo_pull)
        self.trigger.value(1 if not self.trigger_active_high else 0)

    def _send_pulse_and_wait(self):
        self.trigger.value(1 if not self.trigger_active_high else 0)
        time.sleep_us(5)

        self.trigger.value(1 if self.trigger_active_high else 0)
        time.sleep_us(10)
        self.trigger.value(1 if not self.trigger_active_high else 0)

        try:
            pulse_time = machine.time_pulse_us(self.echo, 1, self.echo_timeout_us)
            return pulse_time
        except OSError as ex:
            if len(ex.args) > 0 and ex.args[0] == 110:
                raise OSError("Out of range")
            raise

    def distance_cm(self, filter=True):
        pulse_time = self._send_pulse_and_wait()
        cms = (pulse_time / 2.0) / 29.1

        if cms < 0 or cms > _MAX_DISTANCE_CM:
            cms = _MAX_DISTANCE_CM

        if not filter:
            return cms

        self._ars.append(cms)
        self._ats.append(time.time_ns())
        if len(self._ars) > 5:
            self._ars.pop(0)
            self._ats.pop(0)

        while self._ats and (self._ats[-1] - self._ats[0] > 5e8):
            self._ars.pop(0)
            self._ats.pop(0)

        if len(self._ars) < 2:
            time.sleep_ms(30)
            pulse_time = self._send_pulse_and_wait()
            cms2 = (pulse_time / 2.0) / 29.1
            if cms2 < 0 or cms2 > _MAX_DISTANCE_CM:
                cms2 = _MAX_DISTANCE_CM
            self._ars.append(cms2)
            self._ats.append(time.time_ns())

        N = len(self._ars)
        Fi = [1] * N
        Fd = [1] * N
        maxd = vald = 0
        for i in range(N):
            for j in range(i):
                if (self._ars[i] >= self._ars[j]) and (self._ars[i] - self._ars[j]) < 10:
                    Fi[i] = max(Fi[i], Fi[j] + 1)
                if (self._ars[i] <= self._ars[j]) and (self._ars[j] - self._ars[i]) < 10:
                    Fd[i] = max(Fd[i], Fd[j] + 1)
                if maxd < Fi[i] or maxd < Fd[i]:
                    maxd = max(Fi[i], Fd[i])
                    vald = self._ars[i]

        if maxd <= N / 2:
            vald = sum(self._ars) / N

        return round(vald * 10) / 10

    def distance_mm(self):
        return int(self.distance_cm() * 10)
