from machine import Pin

class Relay:
    def __init__(self, pin, active_high=True, default_off=True):
        self.pin = Pin(int(pin), Pin.OUT)
        self.active_high = bool(active_high)
        if default_off:
            self.off()

    def on(self):
        self.pin.value(1 if self.active_high else 0)

    def off(self):
        self.pin.value(0 if self.active_high else 1)

    def write(self, state):
        self.on() if state else self.off()
