# ble_simple_central.py (MEBLOCK patched)
# Central BLE for Nordic UART Service (NUS)
# Supports "block-style" usage:
#   ble_on_connected(cb) / ble_on_disconnected(cb) / ble_on_rx(cb) can be called BEFORE ble_connect()
# Also supports connect nearest by RSSI.

import bluetooth, json, time
from micropython import const
from ble_advertising import decode_name

# ===== IRQs =====
_IRQ_SCAN_RESULT = const(5)
_IRQ_SCAN_DONE = const(6)
_IRQ_PERIPHERAL_CONNECT = const(7)
_IRQ_PERIPHERAL_DISCONNECT = const(8)
_IRQ_GATTC_SERVICE_RESULT = const(9)
_IRQ_GATTC_SERVICE_DONE = const(10)
_IRQ_GATTC_CHARACTERISTIC_RESULT = const(11)
_IRQ_GATTC_CHARACTERISTIC_DONE = const(12)
_IRQ_GATTC_DESCRIPTOR_RESULT = const(13)
_IRQ_GATTC_DESCRIPTOR_DONE = const(14)
_IRQ_GATTC_WRITE_DONE = const(17)
_IRQ_GATTC_NOTIFY = const(18)

_ADV_IND = const(0x00)
_ADV_DIRECT_IND = const(0x01)

# ===== NUS UUIDs =====
_UART_SERVICE_UUID = bluetooth.UUID("6E400001-B5A3-F393-E0A9-E50E24DCCA9E")
_UART_RX_CHAR_UUID = bluetooth.UUID("6E400002-B5A3-F393-E0A9-E50E24DCCA9E")  # central writes -> peripheral reads
_UART_TX_CHAR_UUID = bluetooth.UUID("6E400003-B5A3-F393-E0A9-E50E24DCCA9E")  # peripheral notifies -> central reads
_CCCD_UUID = bluetooth.UUID(0x2902)

def _get_prefix_default():
    try:
        from setting import DEVICE_NAME_PREFIX
        return DEVICE_NAME_PREFIX
    except:
        return "MEBLOCK-"


class BLESimpleCentral:
    def __init__(self, ble):
        # ensure attrs exist BEFORE any IRQ
        self._on_connected_evt = None
        self._on_disconnected_evt = None
        self._ready_fired = False

        self._ble = ble
        self._ble.active(True)
        self._reset()
        self._ble.irq(self._irq)

    def _reset(self):
        # scan cache
        self._name = None
        self._addr_type = None
        self._addr = None
        self._target_name = None
        self._target_prefix = None

        # callbacks
        self._scan_callback = None
        self._conn_callback = None
        self._notify_callback = None
        self._notify_bytes = False

        # conn/handles
        self._conn_handle = None
        self._start_handle = None
        self._end_handle = None
        self._tx_handle = None
        self._rx_handle = None
        self._cccd_handle = None

        # do NOT wipe event handlers (block-style), only reset fired flag
        self._ready_fired = False

    # event setters
    def on_connected(self, cb):
        self._on_connected_evt = cb

    def on_disconnected(self, cb):
        self._on_disconnected_evt = cb

    def _maybe_fire_connected(self):
        # guard for safety
        if not hasattr(self, "_ready_fired"):
            self._ready_fired = False
        if not hasattr(self, "_on_connected_evt"):
            self._on_connected_evt = None

        if (not self._ready_fired) and self.is_connected():
            self._ready_fired = True
            if self._on_connected_evt:
                try:
                    self._on_connected_evt()
                except:
                    pass

    def _irq(self, event, data):
        if event == _IRQ_SCAN_RESULT:
            addr_type, addr, adv_type, rssi, adv_data = data
            if adv_type not in (_ADV_IND, _ADV_DIRECT_IND):
                return

            name = decode_name(adv_data) or ""
            if not name:
                return

            matched = False
            if self._target_name:
                matched = (name == self._target_name)
            elif self._target_prefix:
                matched = name.startswith(self._target_prefix)

            if matched:
                self._addr_type = addr_type
                self._addr = bytes(addr)
                self._name = name
                try:
                    self._ble.gap_scan(None)
                except:
                    pass

        elif event == _IRQ_SCAN_DONE:
            if self._scan_callback:
                if self._addr:
                    self._scan_callback(self._addr_type, self._addr, self._name)
                else:
                    self._scan_callback(None, None, None)
                self._scan_callback = None

        elif event == _IRQ_PERIPHERAL_CONNECT:
            conn_handle, addr_type, addr = data
            if addr_type == self._addr_type and bytes(addr) == self._addr:
                self._conn_handle = conn_handle
                try:
                    self._ble.gattc_discover_services(self._conn_handle)
                except:
                    pass

        elif event == _IRQ_PERIPHERAL_DISCONNECT:
            conn_handle, _, _ = data
            if conn_handle == self._conn_handle:
                # fire disconnected if we had been ready
                if not hasattr(self, "_on_disconnected_evt"):
                    self._on_disconnected_evt = None
                if not hasattr(self, "_ready_fired"):
                    self._ready_fired = False

                if self._ready_fired and self._on_disconnected_evt:
                    try:
                        self._on_disconnected_evt()
                    except:
                        pass
                self._reset()

        elif event == _IRQ_GATTC_SERVICE_RESULT:
            conn_handle, start_handle, end_handle, uuid = data
            if conn_handle == self._conn_handle and uuid == _UART_SERVICE_UUID:
                self._start_handle, self._end_handle = start_handle, end_handle

        elif event == _IRQ_GATTC_SERVICE_DONE:
            if self._start_handle and self._end_handle:
                try:
                    self._ble.gattc_discover_characteristics(self._conn_handle, self._start_handle, self._end_handle)
                except:
                    pass

        elif event == _IRQ_GATTC_CHARACTERISTIC_RESULT:
            conn_handle, def_handle, value_handle, properties, uuid = data
            if conn_handle != self._conn_handle:
                return
            if uuid == _UART_RX_CHAR_UUID:
                self._rx_handle = value_handle
            elif uuid == _UART_TX_CHAR_UUID:
                self._tx_handle = value_handle
                # discover CCCD
                try:
                    self._ble.gattc_discover_descriptors(conn_handle, def_handle, self._end_handle)
                except:
                    pass

        elif event == _IRQ_GATTC_DESCRIPTOR_RESULT:
            conn_handle, dsc_handle, uuid = data
            if conn_handle == self._conn_handle and uuid == _CCCD_UUID and self._cccd_handle is None:
                self._cccd_handle = dsc_handle

        elif event == _IRQ_GATTC_DESCRIPTOR_DONE:
            # enable notify
            if self._conn_handle and self._cccd_handle:
                try:
                    self._ble.gattc_write(self._conn_handle, self._cccd_handle, b"\x01\x00", 1)
                except:
                    try:
                        self._ble.gattc_write(self._conn_handle, self._cccd_handle, b"\x01\x00", 0)
                    except:
                        pass

        elif event == _IRQ_GATTC_CHARACTERISTIC_DONE:
            # ready
            if self._tx_handle is not None and self._rx_handle is not None:
                if self._conn_callback:
                    cb = self._conn_callback
                    self._conn_callback = None
                    try:
                        cb()
                    except:
                        pass
                self._maybe_fire_connected()

        elif event == _IRQ_GATTC_NOTIFY:
            conn_handle, value_handle, notify_data = data
            if conn_handle == self._conn_handle and value_handle == self._tx_handle:
                if self._notify_callback:
                    if self._notify_bytes:
                        self._notify_callback(bytes(notify_data))
                    else:
                        try:
                            self._notify_callback(notify_data.decode("utf8"))
                        except:
                            self._notify_callback("".join(chr(c) for c in notify_data))

    def is_connected(self):
        return (
            self._conn_handle is not None
            and self._tx_handle is not None
            and self._rx_handle is not None
        )

    def scan(self, callback=None, timeout_ms=4000, target_name=None, target_prefix=None):
        self._addr_type = None
        self._addr = None
        self._name = None
        self._target_name = target_name
        self._target_prefix = target_prefix
        self._scan_callback = callback
        self._ble.gap_scan(timeout_ms, 30000, 30000)

    def connect(self, addr_type=None, addr=None, callback=None):
        self._addr_type = addr_type or self._addr_type
        self._addr = addr or self._addr
        self._conn_callback = callback
        if self._addr_type is None or self._addr is None:
            return False
        self._ble.gap_connect(self._addr_type, self._addr)
        return True

    def disconnect(self):
        if not self._conn_handle:
            return
        try:
            self._ble.gap_disconnect(self._conn_handle)
        except:
            pass
        self._reset()

    def write(self, v, response=False):
        if not self.is_connected():
            return
        if isinstance(v, str):
            v = v.encode()
        self._ble.gattc_write(self._conn_handle, self._rx_handle, v, 1 if response else 0)

    def on_notify(self, callback, as_bytes=False):
        self._notify_callback = callback
        self._notify_bytes = bool(as_bytes)


# ===== Module-level (shared core API) =====
bt = None

# pending handlers allow "block-style" order
_pending_on_rx = None
_pending_on_rx_bytes = False
_pending_on_connected = None
_pending_on_disconnected = None

def _attach_pending_handlers():
    global bt, _pending_on_rx, _pending_on_rx_bytes, _pending_on_connected, _pending_on_disconnected
    if not bt:
        return
    if _pending_on_rx:
        bt.on_notify(_pending_on_rx, as_bytes=_pending_on_rx_bytes)
    if _pending_on_connected:
        bt.on_connected(_pending_on_connected)
    if _pending_on_disconnected:
        bt.on_disconnected(_pending_on_disconnected)

    # if already connected by the time handlers attach, fire connected once
    try:
        bt._maybe_fire_connected()
    except:
        pass


def ble_on_rx(handler, as_bytes=False):
    global bt, _pending_on_rx, _pending_on_rx_bytes
    _pending_on_rx = handler
    _pending_on_rx_bytes = bool(as_bytes)
    if bt:
        bt.on_notify(handler, as_bytes=as_bytes)

def ble_on_connected(handler):
    global bt, _pending_on_connected
    _pending_on_connected = handler
    if bt:
        bt.on_connected(handler)
        try:
            bt._maybe_fire_connected()
        except:
            pass

def ble_on_disconnected(handler):
    global bt, _pending_on_disconnected
    _pending_on_disconnected = handler
    if bt:
        bt.on_disconnected(handler)

def ble_is_connected():
    global bt
    try:
        return bt.is_connected() if bt else False
    except:
        return False


def ble_connect(device=None, prefix=None, timeout_ms=12000):
    global bt
    bt = BLESimpleCentral(bluetooth.BLE())
    _attach_pending_handlers()

    if prefix is None:
        prefix = _get_prefix_default()

    not_found = False

    def on_scan(addr_type, addr, name):
        nonlocal not_found
        if addr_type is not None:
            bt.connect(callback=None)
        else:
            not_found = True

    if device:
        bt.scan(callback=on_scan, timeout_ms=min(timeout_ms, 4000), target_name=device)
    else:
        bt.scan(callback=on_scan, timeout_ms=min(timeout_ms, 4000), target_prefix=prefix)

    t0 = time.ticks_ms()
    while not bt.is_connected():
        time.sleep_ms(100)
        if not_found:
            return False
        if time.ticks_diff(time.ticks_ms(), t0) > timeout_ms:
            return False

    # ensure connected event fires (block-style)
    try:
        bt._maybe_fire_connected()
    except:
        pass
    return True


def ble_connect_nearest(prefix=None, scan_ms=4000, timeout_ms=12000):
    global bt
    bt = BLESimpleCentral(bluetooth.BLE())
    _attach_pending_handlers()

    if prefix is None:
        prefix = _get_prefix_default()

    best_rssi = -999
    best_addr_type = None
    best_addr = None
    best_name = None
    scan_done = False

    def _irq_tmp(event, data):
        nonlocal best_rssi, best_addr_type, best_addr, best_name, scan_done
        if event == _IRQ_SCAN_RESULT:
            addr_type, addr, adv_type, rssi, adv_data = data
            if adv_type not in (_ADV_IND, _ADV_DIRECT_IND):
                return
            name = decode_name(adv_data) or ""
            if not name.startswith(prefix):
                return
            if rssi > best_rssi:
                best_rssi = rssi
                best_addr_type = addr_type
                best_addr = bytes(addr)
                best_name = name
        elif event == _IRQ_SCAN_DONE:
            scan_done = True

    bt._ble.irq(_irq_tmp)
    bt._ble.gap_scan(scan_ms, 30000, 30000)

    t0 = time.ticks_ms()
    while not scan_done:
        time.sleep_ms(50)
        if time.ticks_diff(time.ticks_ms(), t0) > (scan_ms + 2000):
            break

    bt._ble.irq(bt._irq)

    if best_addr is None:
        return False

    bt._addr_type = best_addr_type
    bt._addr = best_addr
    bt._name = best_name

    bt.connect(callback=None)

    t1 = time.ticks_ms()
    while not bt.is_connected():
        time.sleep_ms(100)
        if time.ticks_diff(time.ticks_ms(), t1) > timeout_ms:
            return False

    try:
        bt._maybe_fire_connected()
    except:
        pass
    return True


def ble_send(name, value=None):
    global bt
    if not bt:
        return
    if value is None:
        bt.write(str(name))
    else:
        bt.write(json.dumps((name, value)))

def ble_disconnect():
    global bt
    if bt:
        bt.disconnect()
