# ble_uart_repl.py
# BLE REPL over NUS (Peripheral)
# Patched:
# - write() returns count (avoid dupterm resend)
# - expose UART via get_uart()

import io, os
from micropython import const
from ble_uart_peripheral import BLEUART

_MP_STREAM_POLL = const(3)
_MP_STREAM_POLL_RD = const(0x0001)

_uart = None

def get_uart():
    return _uart

class BLEUARTRepl(io.IOBase):
    def __init__(self, uart: BLEUART):
        self._uart = uart
        try:
            self._uart.irq(self._on_rx)  # REPL handler
        except:
            pass

    def read(self, sz=None):
        return self._uart.read(sz)

    def readinto(self, buf):
        data = self._uart.read(len(buf))
        if not data:
            return None
        n = len(data)
        buf[:n] = data
        return n

    def write(self, buf):
        if not buf:
            return 0
        mv = memoryview(buf)
        sent = 0
        for i in range(0, len(mv), 20):
            chunk = mv[i:i+20]
            try:
                self._uart.write(chunk)
                sent += len(chunk)
            except:
                pass
        return sent

    def ioctl(self, op, arg):
        if op == _MP_STREAM_POLL:
            return _MP_STREAM_POLL_RD if self._uart.any() else 0
        return 0

    def _on_rx(self):
        try:
            os.dupterm_notify(None)
        except:
            pass

def start(name="MEBLOCK-REPL"):
    global _uart
    from bluetooth import BLE
    ble = BLE()
    uart = BLEUART(ble, name=name, rxbuf=256)
    _uart = uart
    os.dupterm(BLEUARTRepl(uart))
    print("[BLE] REPL ready:", name)
