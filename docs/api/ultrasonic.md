# `ultrasonic` — Ultrasonic Module (HC‑SR04)

Chức năng chính và chức năng của `ultrasonic`.

## Function

```sig
HCSR04(trigger_pin, echo_pin)
```
Khởi tạo cảm biến với chân TRIG/ECHO, hoặc truyền tên board có chân định sẵn.

    TRIG (có thể truyền số, chuỗi như "2", "GPIO2", "IO2", "P2", hoặc key tên trong pinmap như "TRIG")

    ECHO tương tự.

```sig
distance_cm(filter=True)
```
Đo khoảng cách và trả về giá trị (cm) dưới dạng float (với 1 chữ số thập phân).

    filter (bool, mặc định True): bật/tắt bộ lọc nhiễu nội bộ.

```sig
distance_mm(filter=True)
```
 Trả về khoảng cách theo mm (đơn vị milimét).

    filter (bool, mặc định True): bật/tắt bộ lọc nhiễu nội bộ.
Gọi distance_cm() rồi chuyển sang mm (nhân 10) và cast về int.
## Sample Code

```python
from ultrasonic import HCSR04
sensor = HCSR04(2, 3)
print(sensor.distance_cm(), "cm")
```
