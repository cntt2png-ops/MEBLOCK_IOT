# `oled_i2c` — OLED I2C Module

Chức năng chính và chức năng của **oled_i2c** (SSD1306/SH1106).

## Function

`oled = Oled(board="s3", driver="SH1106", sh1106_offset=2)`  
Khởi tạo OLED đã được kết nối bằng **I2C**. `board`: preset chân; `driver`: `SSD1306` hoặc `SH1106`.

`oled.text(TEXT, X, Y)`  
Thiết lập nội dung (TEXT) lên màn hình tại vị trí mong muốn. X: 0..127, Y: 0..63.

`oled.show()`  
Lệnh hiển thị nội dung.

`oled.fill(0)`  
Xóa nội dung trên màn hình (`1` = trắng toàn màn).

!!! note "Tùy chọn khác"
    Dùng **factory**:  
    ```python
    from oled import create
    dev = create(ctrl="SSD1306", i2c_sda=21, i2c_scl=22, addr=0x3C, backend="soft")
    ```
    Với **SH1106** (132 cột), có thể cần `sh1106_offset=2` để canh 128 cột vào giữa.

## Sample Code

Hiển thị ký tự trên màn hình OLED:

```python
from oled import Oled
oled = Oled(board="s3", driver="SH1106")  # hoặc driver="SSD1306"

oled.clear()
oled.text("MEBLOCK IOT", 0, 0)
oled.text("Hello, ESP32!", 0, 16)
oled.show()
```
