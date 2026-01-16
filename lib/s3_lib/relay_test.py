from relay_1chs import Relay
import time

relay = Relay(pin=4)  # GPIO5, relay tích cực dương

relay.on()
time.sleep(2)
relay.off()
