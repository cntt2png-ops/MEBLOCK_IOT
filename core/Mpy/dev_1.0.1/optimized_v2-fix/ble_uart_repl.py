# ble_uart_repl.py
# BLE UART REPL for MicroPython over Nordic UART Service (NUS).

import io
import os
import micropython

from micropython import const
from ble_uart_peripheral import BLEUART

try:
    from onboard import send_inf, send_war
except Exception:
    def send_inf(code_or_text, text=None, out=None):
        try:
            if text is None:
                print(code_or_text)
            else:
                print("[%s] %s" % (code_or_text, text))
        except Exception:
            pass

    def send_war(code_or_text, text=None, out=None):
        try:
            if text is None:
                print(code_or_text)
            else:
                print("[%s] %s" % (code_or_text, text))
        except Exception:
            pass

_MP_STREAM_POLL = const(3)
_MP_STREAM_POLL_RD = const(0x0001)
_DEFAULT_CHUNK = const(20)

_ble = None
_uart = None
_repl = None


def _scheduled_info(payload):
    try:
        code, text = payload
        send_inf(code, text)
    except Exception:
        pass


def _scheduled_warn(payload):
    try:
        code, text = payload
        send_war(code, text)
    except Exception:
        pass


def _defer_info(code, text):
    try:
        micropython.schedule(_scheduled_info, (code, text))
    except Exception:
        try:
            send_inf(code, text)
        except Exception:
            pass


def _defer_warn(code, text):
    try:
        micropython.schedule(_scheduled_warn, (code, text))
    except Exception:
        try:
            send_war(code, text)
        except Exception:
            pass


class BLEUARTRepl(io.IOBase):
    def __init__(self, uart):
        self._uart = uart
        try:
            self._uart.irq(self._on_rx)
        except Exception:
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

        nb = 0
        mv = memoryview(buf)
        for i in range(0, len(mv), _DEFAULT_CHUNK):
            chunk = mv[i:i + _DEFAULT_CHUNK]
            try:
                self._uart.write(chunk)
                nb += len(chunk)
            except Exception:
                nb += len(chunk)
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


def get_uart():
    return _uart


def stop():
    global _ble, _uart, _repl

    try:
        os.dupterm(None)
    except Exception:
        pass

    if _uart:
        try:
            _uart.close()
        except Exception:
            pass

    _ble = None
    _uart = None
    _repl = None


def start(name="MEBLOCK-TOPKID", rxbuf=256):
    global _ble, _uart, _repl

    if _uart and _repl:
        try:
            _uart.set_name(name)
        except Exception:
            _defer_warn("W_BLE_NAME", "Cannot update BLE name")
        try:
            os.dupterm(_repl)
        except Exception:
            pass
        _defer_info("I_BLE_REPL", "REPL resumed as: %s" % (_uart.name() or name))
        return _uart

    from bluetooth import BLE

    _ble = BLE()
    _uart = BLEUART(_ble, name=name, rxbuf=rxbuf)

    def _on_connect(conn_handle):
        _defer_info("I_BLE_CONN", "connected:%s" % conn_handle)

    def _on_disconnect(conn_handle):
        _defer_warn("W_BLE_CONN", "disconnected:%s" % conn_handle)

    try:
        _uart.on_connect(_on_connect)
        _uart.on_disconnect(_on_disconnect)
    except Exception:
        pass

    _repl = BLEUARTRepl(_uart)
    os.dupterm(_repl)
    _defer_info("I_BLE_REPL", "REPL started as: %s" % (_uart.name() or name))
    return _uart


if __name__ == "__main__":
    start()
