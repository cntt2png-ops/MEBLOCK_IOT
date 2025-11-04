# `aiot_dht` — DHT Sensors (DHT20/AHT20 & DHT11)

Một API chung để đọc **nhiệt độ/độ ẩm** từ DHT20/AHT20 (I2C) và DHT11 (1‑wire).

## Function

`AIOT_DHT(sensor="dht20"|"aht20"|"dht11", *, board=None, sda=None, scl=None, pin=None)`  
Tạo đối tượng cảm biến. I2C (DHT20/AHT20) dùng `sda/scl` hoặc preset `board`; DHT11 dùng `pin`.

`read() -> bool`  
Cập nhật dữ liệu. Trả về `True` nếu thành công.

`temperature() -> float`  
Nhiệt độ (°C).

`humidity() -> float`  
Độ ẩm (%RH).

`status() -> dict`  
Thông tin gỡ lỗi (raw, lỗi gần nhất…).

## Sample Code

```python
from aiot_dht import AIOT_DHT

d = AIOT_DHT(sensor="dht20", board="s3")  # hoặc truyền sda/scl
if d.read():
    print("T:", d.temperature(), "C  H:", d.humidity(), "%")
else:
    print("Read fail")
```
