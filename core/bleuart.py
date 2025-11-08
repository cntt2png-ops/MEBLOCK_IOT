# bleuart.py — BLE UART (Nordic UART Service) tối giản cho MicroPython ESP32
import struct
import bluetooth
import time

# UUID Nordic UART Service
_UUID_NUS    = bluetooth.UUID("6E400001-B5A3-F393-E0A9-E50E24DCCA9E")
_UUID_NUS_RX = bluetooth.UUID("6E400002-B5A3-F393-E0A9-E50E24DCCA9E")  # Write / Write No Resp
_UUID_NUS_TX = bluetooth.UUID("6E400003-B5A3-F393-E0A9-E50E24DCCA9E")  # Notify

_FLAG_READ      = bluetooth.FLAG_READ
_FLAG_WRITE     = bluetooth.FLAG_WRITE
_FLAG_WRITE_NR  = bluetooth.FLAG_WRITE_NO_RESPONSE
_FLAG_NOTIFY    = bluetooth.FLAG_NOTIFY

_IRQ_CENTRAL_CONNECT    = 1
_IRQ_CENTRAL_DISCONNECT = 2
_IRQ_GATTS_WRITE        = 3

def _adv_payload(name=None, services=None):
    """Tạo payload Advertising/Scan Response tối giản, tránh vượt 31B."""
    payload = bytearray()
    if name:
        nb = name.encode()
        payload += struct.pack("BB", len(nb) + 1, 0x09) + nb  # Complete Local Name
    if services:
        for uuid in services:
            b = bytes(uuid)
            if len(b) == 2:
                payload += struct.pack("BB", 3, 0x03) + b  # 16-bit Complete List
            elif len(b) == 16:
                payload += struct.pack("BB", 17, 0x07) + b  # 128-bit Complete List
    return payload

class BLEUART:
    def __init__(self, name="MEBLOCK-TOPKID", rx_callback=None):
        self._ble = bluetooth.BLE()
        self._ble.active(True)
        self._ble.irq(self._irq)

        self._name = name
        self._rx_cb = rx_callback
        self._conn = None

        # GATT: NUS service với 2 đặc tính (TX notify, RX write)
        tx_char = (_UUID_NUS_TX, _FLAG_NOTIFY)
        rx_char = (_UUID_NUS_RX, _FLAG_WRITE | _FLAG_WRITE_NR)
        nus_service = (_UUID_NUS, (tx_char, rx_char))
        ((self._tx_handle, self._rx_handle),) = self._ble.gatts_register_services((nus_service,))

        # Tăng buffer GATT để truyền file (an toàn nếu FW hỗ trợ)
        try:
            self._ble.gatts_set_buffer(self._tx_handle, 512, True)
            self._ble.gatts_set_buffer(self._rx_handle, 512, True)
        except Exception as e:
            print("[BLEUART] set_buffer error:", e)

        self.advertise(True)

    # ====== IRQ ======
    def _irq(self, event, data):
        if event == _IRQ_CENTRAL_CONNECT:
            self._conn, addr_type, addr = data
            print("[BLEUART] Connected:", self._conn)
        elif event == _IRQ_CENTRAL_DISCONNECT:
            ch, addr_type, addr = data
            print("[BLEUART] Disconnected:", ch)
            self._conn = None
            self.advertise(True)
        elif event == _IRQ_GATTS_WRITE:
            ch, attr = data
            if attr == self._rx_handle:
                buf = self._ble.gatts_read(self._rx_handle)
                if self._rx_cb:
                    try:
                        self._rx_cb(buf)
                    except Exception as e:
                        print("[BLEUART] RX callback error:", e)

    # ====== GAP advertise (đã vá: chia adv & scan response) ======
    def advertise(self, enable=True, interval_us=500000):
        if not enable:
            self._ble.gap_advertise(None)
            return
        adv = _adv_payload(name=None, services=[_UUID_NUS])   # chỉ services
        sr  = _adv_payload(name=self._name, services=None)    # chỉ name
        try:
            self._ble.gap_advertise(interval_us, adv_data=adv, resp_data=sr)
            print("[BLEUART] Advertising as:", self._name)
        except Exception as e:
            print("[BLEUART] advertise error:", e)
            # Fallback: chỉ name (đảm bảo vẫn phát sóng)
            try:
                only_name = _adv_payload(name=self._name, services=None)
                self._ble.gap_advertise(interval_us, adv_data=only_name)
                print("[BLEUART] Advertising (name only).")
            except Exception as e2:
                print("[BLEUART] advertise fallback error:", e2)

    # ====== Helpers ======
    def is_connected(self):
        return self._conn is not None

    def send(self, data):
        """Notify TX; cắt nhỏ <= 20B cho an toàn."""
        if not self.is_connected():
            return 0
        if isinstance(data, str):
            data = data.encode()
        sent = 0
        for i in range(0, len(data), 20):
            chunk = data[i:i+20]
            try:
                self._ble.gatts_notify(self._conn, self._tx_handle, chunk)
                sent += len(chunk)
                # nghỉ nhẹ để tránh nghẽn (tùy FW)
                # time.sleep_ms(1)
            except Exception as e:
                print("[BLEUART] notify error:", e)
                break
        return sent
