# ble_uart_peripheral.py
# Nordic UART Service (NUS) peripheral with low-fragmentation RX buffering.

import bluetooth
from micropython import const

from ble_advertising import advertising_payload, fit_name_for_advertising

_IRQ_CENTRAL_CONNECT = const(1)
_IRQ_CENTRAL_DISCONNECT = const(2)
_IRQ_GATTS_WRITE = const(3)

_FLAG_WRITE = const(0x0008)
_FLAG_NOTIFY = const(0x0010)

_UART_UUID = bluetooth.UUID("6E400001-B5A3-F393-E0A9-E50E24DCCA9E")
_UART_TX = (
    bluetooth.UUID("6E400003-B5A3-F393-E0A9-E50E24DCCA9E"),
    _FLAG_NOTIFY,
)
_UART_RX = (
    bluetooth.UUID("6E400002-B5A3-F393-E0A9-E50E24DCCA9E"),
    _FLAG_WRITE,
)
_UART_SERVICE = (
    _UART_UUID,
    (_UART_TX, _UART_RX),
)

_ADV_APPEARANCE_GENERIC_COMPUTER = const(128)


class _ByteRingBuffer:
    def __init__(self, size):
        size = int(size or 0)
        if size < 32:
            size = 32
        self._size = size
        self._buf = bytearray(size)
        self._head = 0
        self._tail = 0
        self._count = 0
        self._dropped = 0

    def __len__(self):
        return self._count

    def clear(self):
        self._head = 0
        self._tail = 0
        self._count = 0

    def dropped(self):
        return self._dropped

    def write(self, data):
        if not data:
            return 0
        if not isinstance(data, (bytes, bytearray, memoryview)):
            data = bytes(data)

        written = 0
        for b in data:
            if self._count >= self._size:
                # discard oldest byte to keep recent commands/data
                self._tail = (self._tail + 1) % self._size
                self._count -= 1
                self._dropped += 1
            self._buf[self._head] = b
            self._head = (self._head + 1) % self._size
            self._count += 1
            written += 1
        return written

    def read(self, size=None):
        if self._count <= 0:
            return b""

        if size is None or size <= 0 or size > self._count:
            size = self._count

        out = bytearray(size)
        for i in range(size):
            out[i] = self._buf[self._tail]
            self._tail = (self._tail + 1) % self._size
        self._count -= size
        return bytes(out)


class BLEUART:
    def __init__(self, ble, name="mpy-uart", rxbuf=256, append_gatts_buffer=256):
        self._ble = ble
        self._ble.active(True)
        self._ble.irq(self._irq)

        ((self._tx_handle, self._rx_handle),) = self._ble.gatts_register_services((_UART_SERVICE,))
        self._ble.gatts_set_buffer(self._rx_handle, max(64, int(append_gatts_buffer or rxbuf or 256)), True)

        self._connections = set()
        self._rx_buffer = _ByteRingBuffer(rxbuf)
        self._handler = None
        self._connect_handler = None
        self._disconnect_handler = None
        self._name = None
        self._payload = None
        self._advertising = False

        self.set_name(name)
        self._advertise()

    def irq(self, handler):
        self._handler = handler

    def on_connect(self, handler):
        self._connect_handler = handler

    def on_disconnect(self, handler):
        self._disconnect_handler = handler

    def set_name(self, name):
        safe_name = fit_name_for_advertising(name, appearance=_ADV_APPEARANCE_GENERIC_COMPUTER)
        self._name = safe_name.decode("utf-8") if safe_name else ""
        self._payload = advertising_payload(
            name=safe_name,
            appearance=_ADV_APPEARANCE_GENERIC_COMPUTER,
        )
        if not self._connections:
            self._advertise()

    def name(self):
        return self._name

    def is_connected(self):
        return len(self._connections) > 0

    def connections(self):
        return len(self._connections)

    def any(self):
        return len(self._rx_buffer)

    def dropped(self):
        return self._rx_buffer.dropped()

    def read(self, sz=None):
        return self._rx_buffer.read(sz)

    def write(self, data):
        if not data:
            return 0

        if isinstance(data, str):
            data = data.encode("utf-8")
        elif isinstance(data, bytearray):
            data = bytes(data)

        sent = 0
        for conn_handle in tuple(self._connections):
            try:
                self._ble.gatts_notify(conn_handle, self._tx_handle, data)
                sent += len(data)
            except Exception:
                pass
        return sent

    def flush_rx(self):
        self._rx_buffer.clear()

    def close(self):
        self.stop_advertise()
        for conn_handle in tuple(self._connections):
            try:
                self._ble.gap_disconnect(conn_handle)
            except Exception:
                pass
        self._connections.clear()
        self.flush_rx()

    def stop_advertise(self):
        try:
            self._ble.gap_advertise(None)
        except Exception:
            pass
        self._advertising = False

    def _advertise(self, interval_us=500000):
        try:
            self._ble.gap_advertise(interval_us, adv_data=self._payload)
            self._advertising = True
        except Exception:
            self._advertising = False

    def _handle_rx_irq(self):
        try:
            data = self._ble.gatts_read(self._rx_handle)
        except Exception:
            data = b""

        if data:
            self._rx_buffer.write(data)

        if self._handler:
            try:
                self._handler()
            except Exception:
                pass

    def _irq(self, event, data):
        if event == _IRQ_CENTRAL_CONNECT:
            conn_handle, _, _ = data
            self._connections.add(conn_handle)
            self._advertising = False
            if self._connect_handler:
                try:
                    self._connect_handler(conn_handle)
                except Exception:
                    pass

        elif event == _IRQ_CENTRAL_DISCONNECT:
            conn_handle, _, _ = data
            if conn_handle in self._connections:
                self._connections.remove(conn_handle)

            if self._disconnect_handler:
                try:
                    self._disconnect_handler(conn_handle)
                except Exception:
                    pass

            if not self._connections:
                self._advertise()

        elif event == _IRQ_GATTS_WRITE:
            conn_handle, value_handle = data
            if conn_handle in self._connections and value_handle == self._rx_handle:
                self._handle_rx_irq()


def demo():
    import time

    ble = bluetooth.BLE()
    uart = BLEUART(ble)

    def on_rx():
        try:
            print("rx:", uart.read().decode().strip())
        except Exception:
            print("rx:", uart.read())

    uart.irq(on_rx)
    nums = [4, 8, 15, 16, 23, 42]
    i = 0

    try:
        while True:
            uart.write(str(nums[i]) + "\n")
            i = (i + 1) % len(nums)
            time.sleep_ms(1000)
    except KeyboardInterrupt:
        pass

    uart.close()


if __name__ == "__main__":
    demo()
