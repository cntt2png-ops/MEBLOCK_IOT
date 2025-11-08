# `dht` — DHT Sensors

Chức năng chính và chức năng của `dht`.

## Function

```sig
DHT(sensor, *, board=None, sda=None, scl=None, pin=None)
```
Khởi tạo đối tượng điều khiển cảm biến nhiệt độ/độ ẩm: 

    sensor (str): Loại cảm biến

        "dht20": Cho cảm biến DHT20/AHT20 (I2C)

            board: Preset board tự chọn chân

            sda: Chân SDA 

            scl: Chân SCL 

            addr: Địa chỉ I2C (mặc định: 0x38)

        "dht11/22": Cho cảm biến DHT11/22 (1-wire)

            pin: Số chân GPIO 

```sig
read() -> bool
```
Đọc dữ liệu từ cảm biến
`True`{.pill .pill-true} nếu đọc thành công, `False`{.pill .pill-false} nếu lỗi.
Kết quả được lưu trong bộ nhớ đệm cho các hàm temperature()/humidity()

```sig
temperature() -> float • humidity() -> float
```
Đọc giá trị nhiệt độ (°C)

    DHT20: Giá trị từ -50°C đến 85°C, độ phân giải 0.01°C

    DHT11: Giá trị từ 0°C đến 50°C, độ phân giải 1°C

Tự động gọi read() nếu chưa có dữ liệu

```sig
humidity() -> float
```
Đọc giá trị độ ẩm tương đối (%)

    DHT20: Giá trị từ 0% đến 100%, độ phân giải 0.024%
    
    DHT11: Giá trị từ 20% đến 90%, độ phân giải 1%

Tự động gọi read() nếu chưa có dữ liệu
## Sample Code

```python
from dht import DHT
d = DHT("dht20", board="s3")
if d.read():
    print(d.temperature(), d.humidity())
```
