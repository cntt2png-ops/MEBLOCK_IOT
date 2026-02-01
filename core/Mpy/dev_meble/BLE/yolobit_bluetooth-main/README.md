# yolobit_bluetooth
Extension library for bluetooh support for Yolo:Bit. Current support 3 modes

## Simple peripheral mode
Yolo:Bit starts bluetooth as a peripheral device (like smart watch, beacon...) so that other device or phone app can connect to.

```python
from ble_uart_peripheral import *
ble_start()
```

## Simple central mode
Yolo:Bit starts bluetooth to connect to other bluetooth device.

```python
from ble_simple_central import *
ble_start()
```

## REPL mode
Yolo:Bit starts bluetooth and turn on terminal mode so that phone app can connect and run code directly.

```python
from ble_uart_repl import *
ble_start()
```

# Send message
After bluetooth connection is setup between 2 Yolo:Bit, they can send data to each other
## Send a simple message
```python
ble_send('hello')
```
## Send message with format name = value
```python
ble_send('name', 'value')
ble_send('color', 'red')
ble_send('temperature', '25.5')
```

# Receive message
## Receive a string
```python
def ble_on_rx_callback(msg):
    print('Received: ', msg)

ble_on_rx(ble_on_rx_callback)
```

## Receive a number
```python
def ble_on_rx_callback(msg):
    number = int(msg) if msg.isdigit() else None # on received number 
    print('Received number: ', number)

ble_on_rx(ble_on_rx_callback)
```

## Receive a name=value message
```python
import json
def ble_on_rx_callback(msg):
    name = None
    value = None
    try:
        name, value = json.loads(msg)
    except:
        pass
    print(name, ': ', value)

ble_on_rx(ble_on_rx_callback)
```