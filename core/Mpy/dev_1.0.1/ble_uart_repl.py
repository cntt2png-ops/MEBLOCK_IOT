# ble_uart_repl.py - BLE UART REPL for MicroPython
# Simple dupterm stream over Nordic UART Service (NUS)
# Works with ble_uart_peripheral.BLEUART
#
# Key fix: write() returns the count of bytes accepted so os.dupterm()
#          does NOT retry and spam-resend chunks.
#
# Added:
# - expose the BLEUART instance used by REPL via get_uart()
#   so main.py can attach additional RX callback (append=True)
#   and can send data using uart.write(...)

import io
import os
from micropython import const
from ble_uart_peripheral import BLEUART

_MP_STREAM_POLL = const(3)
_MP_STREAM_POLL_RD = const(0x0001)

# ===== expose UART used by REPL =====
_uart = None

def get_uart():
    return _uart


class BLEUARTRepl(io.IOBase):
    def __init__(self, uart: BLEUART):
        self._uart = uart
        # wake dupterm when data arrives
        try:
            self._uart.irq(self._on_rx)  # keep compatibility (single handler for REPL)
        except Exception:
            pass

    # ===== dupterm hooks =====
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
        # Chunk at 20 bytes for NUS. Always report progress (len(chunk)).
        # DO NOT return 0, or dupterm will retry indefinitely.
        if not buf:
            return 0
        nb = 0
        mv = memoryview(buf)
        for i in range(0, len(mv), 20):
            chunk = mv[i:i + 20]
            try:
                self._uart.write(chunk)   # may return None
                nb += len(chunk)
            except Exception:
                pass
        return nb

    def ioctl(self, op, arg):
        if op == _MP_STREAM_POLL:
            if self._uart.any():
                return _MP_STREAM_POLL_RD
            return 0
        return 0

    def _on_rx(self):
        try:
            os.dupterm_notify(None)
        except Exception:
            pass


def start(name="MEBLOCK-TOPKID"):
    global _uart
    from bluetooth import BLE
    ble = BLE()
    uart = BLEUART(ble, name=name)
    _uart = uart
    os.dupterm(BLEUARTRepl(uart))
    print("[BLEUART] REPL started as:", name)


if __name__ == "__main__":
    start()
