# ble_uart_peripheral.py
# Nordic UART Service (NUS) - Peripheral
# Patched for shared-core API:
# - multi RX handlers: irq(handler, append=True)
# - connection events: on_connected(cb), on_disconnected(cb)
# - module-level shared API: ble_on_rx, ble_on_connected, ble_on_disconnected, ble_send, ble_is_connected

import bluetooth
from ble_advertising import advertising_payload
from micropython import const

_IRQ_CENTRAL_CONNECT = const(1)
_IRQ_CENTRAL_DISCONNECT = const(2)
_IRQ_GATTS_WRITE = const(3)

_FLAG_WRITE = const(0x0008)
_FLAG_NOTIFY = const(0x0010)

_UART_UUID = bluetooth.UUID("6E400001-B5A3-F393-E0A9-E50E24DCCA9E")
_UART_TX = (bluetooth.UUID("6E400003-B5A3-F393-E0A9-E50E24DCCA9E"), _FLAG_NOTIFY)
_UART_RX = (bluetooth.UUID("6E400002-B5A3-F393-E0A9-E50E24DCCA9E"), _FLAG_WRITE)
_UART_SERVICE = (_UART_UUID, (_UART_TX, _UART_RX))

_ADV_APPEARANCE_GENERIC_COMPUTER = const(128)


class BLEUART:
    def __init__(self, ble, name="mpy-uart", rxbuf=256):
        self._ble = ble
        self._ble.active(True)
        self._ble.irq(self._irq)

        ((self._tx_handle, self._rx_handle),) = self._ble.gatts_register_services((_UART_SERVICE,))
        self._ble.gatts_set_buffer(self._rx_handle, rxbuf, True)

        self._connections = set()
        self._rx_buffer = bytearray()

        # multi RX handlers
        self._handlers = []

        # connection events
        self._on_connected = None
        self._on_disconnected = None

        self._payload = advertising_payload(name=name, appearance=_ADV_APPEARANCE_GENERIC_COMPUTER)
        self._advertise()

    # RX callback(s)
    def irq(self, handler, append=False):
        """
        append=False: set single handler (compatible)
        append=True : add handler, keep existing (for BLE REPL + app)
        """
        if handler is None:
            self._handlers = []
            return
        if append:
            self._handlers.append(handler)
        else:
            self._handlers = [handler]

    def on_connected(self, cb):
        self._on_connected = cb

    def on_disconnected(self, cb):
        self._on_disconnected = cb

    def _irq(self, event, data):
        if event == _IRQ_CENTRAL_CONNECT:
            conn_handle, _, _ = data
            self._connections.add(conn_handle)
            if self._on_connected:
                try:
                    self._on_connected(conn_handle)
                except:
                    pass

        elif event == _IRQ_CENTRAL_DISCONNECT:
            conn_handle, _, _ = data
            if conn_handle in self._connections:
                self._connections.remove(conn_handle)
            if self._on_disconnected:
                try:
                    self._on_disconnected(conn_handle)
                except:
                    pass
            self._advertise()

        elif event == _IRQ_GATTS_WRITE:
            conn_handle, value_handle = data
            if conn_handle in self._connections and value_handle == self._rx_handle:
                self._rx_buffer += self._ble.gatts_read(self._rx_handle)
                for h in self._handlers:
                    try:
                        h()
                    except:
                        pass

    def any(self):
        return len(self._rx_buffer)

    def read(self, sz=None):
        if not sz:
            sz = len(self._rx_buffer)
        out = self._rx_buffer[:sz]
        self._rx_buffer = self._rx_buffer[sz:]
        return out

    def write(self, data):
        for ch in self._connections:
            try:
                self._ble.gatts_notify(ch, self._tx_handle, data)
            except:
                pass

    def close(self):
        for ch in list(self._connections):
            try:
                self._ble.gap_disconnect(ch)
            except:
                pass
        self._connections.clear()

    def _advertise(self, interval_us=500000):
        self._ble.gap_advertise(interval_us, adv_data=self._payload)


# =========================
# Shared-core module-level API (Peripheral role)
# =========================
_uart = None

def ble_init(name="MEBLOCK-A", rxbuf=256):
    """Only call if you are NOT using BLE REPL. If BLE REPL is used, it already created BLEUART."""
    global _uart
    if _uart:
        return _uart
    ble = bluetooth.BLE()
    _uart = BLEUART(ble, name=name, rxbuf=rxbuf)
    return _uart

def ble_bind_uart(uart_obj):
    """Bind existing BLEUART instance (e.g. from BLE REPL) to shared API."""
    global _uart
    _uart = uart_obj
    return _uart

def ble_on_rx(cb, append=True):
    """cb(data_bytes) called when central writes."""
    if not _uart:
        return
    def _h():
        d = _uart.read()
        if d:
            try:
                cb(d)
            except:
                pass
    _uart.irq(_h, append=append)

def ble_on_connected(cb):
    if not _uart or not hasattr(_uart, "on_connected"):
        return
    _uart.on_connected(lambda ch: cb())

def ble_on_disconnected(cb):
    if not _uart or not hasattr(_uart, "on_disconnected"):
        return
    _uart.on_disconnected(lambda ch: cb())

def ble_send(data):
    if not _uart:
        return
    if isinstance(data, str):
        data = data.encode()
    _uart.write(data)

def ble_is_connected():
    try:
        return _uart is not None and hasattr(_uart, "_connections") and (len(_uart._connections) > 0)
    except:
        return False
