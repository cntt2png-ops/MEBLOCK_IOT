Optimized corefiles for MicroPython BLE / boot / storage.

Main changes:
- BLE RX buffer changed from repeated bytearray concatenation to circular buffer.
- BLE REPL start() is idempotent and has stop().
- Device name is cached and saved atomically.
- Advertising name is auto-trimmed to fit 31-byte BLE advertisement payload.
- Onboard RGB is lazy-initialized instead of creating NeoPixel immediately on import.
- boot.py uses IRQ-based BOOT double-press detection to reduce CPU polling cost.

Files included:
- ble_advertising.py
- ble_uart_peripheral.py
- ble_uart_repl.py
- boot.py
- onboard.py
- setting.py
- utility.py
