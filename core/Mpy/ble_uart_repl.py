# ble_uart_repl.py - BLE UART REPL for MicroPython
# Simple dupterm stream over Nordic UART Service (NUS)
# Works with ble_uart_peripheral.BLEUART
#
# Key fix: write() returns the count of bytes accepted so os.dupterm()
#          does NOT retry and spam-resend chunks.

import io
import os
from micropython import const
from ble_uart_peripheral import BLEUART

_MP_STREAM_POLL = const(3)
_MP_STREAM_POLL_RD = const(0x0001)

class BLEUARTRepl(io.IOBase):
    def __init__(self, uart: BLEUART):
        self._uart = uart
        # wake dupterm when data arrives
        try:
            self._uart.irq(self._on_rx)
        except Exception:
            # older helpers may not have irq(); safe to ignore
            pass

    # ===== dupterm hooks =====
    def read(self, sz=None):
        # Return up to sz bytes; None/0 => non-blocking
        return self._uart.read(sz)

    def readinto(self, buf):
        data = self._uart.read(len(buf))
        if not data:
            return None
        n = len(data)
        buf[:n] = data
        return n

    def write(self, buf):
        # Chunk at 20 bytes for NUS. Always report progress (len(chunk)).
        # DO NOT return 0, or dupterm will retry indefinitely.
        if not buf:
            return 0
        nb = 0
        mv = memoryview(buf)
        for i in range(0, len(mv), 20):
            chunk = mv[i:i+20]
            try:
                self._uart.write(chunk)   # may return None
                nb += len(chunk)
            except Exception:
                # swallow transient radio errors; report bytes we *intended* to push
                pass
        return nb

    def ioctl(self, op, arg):
        # allow VM to poll for input
        if op == _MP_STREAM_POLL:
            if self._uart.any():
                return _MP_STREAM_POLL_RD
            return 0
        return 0

    # ===== internal =====
    def _on_rx(self):
        try:
            os.dupterm_notify(None)
        except Exception:
            pass


def start(name="MEBLOCK-TOPKID"):
    # Create BLEUART and attach dupterm stream
    from bluetooth import BLE
    ble = BLE()
    uart = BLEUART(ble, name=name)
    os.dupterm(BLEUARTRepl(uart))
    print("[BLEUART] REPL started as:", name)

if __name__ == "__main__":
    start()
