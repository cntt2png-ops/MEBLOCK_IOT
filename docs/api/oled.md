# `oled` — OLED I2C Module (SSD1306/SH1106)

Chức năng chính và chức năng của `oled`.

## Function

```sig
Oled( board="s3", width=128, height=64, driver="SH1106", sda=21, scl=22, addr=0x3C)
```
Tạo một thiết bị OLED để sử dụng với thông số:   

    board có định nghĩa sẵn chân, nếu chọn board thì không cần chọn chân thêm

    kích thước: width x height 

    driver: SH1106 hoặc SSD1306 tùy loại OLED

    addr: địa chỉ chọn cố định theo thiết bị hoặc tư dò (bỏ trống)
```sig
show()
```
Đẩy buffer lên màn hình. 

    Sau các thao tác vẽ hay xử lý chuỗi, phải gọi show() thì mới hiển thị.

```sig
clear()
```
Tô toàn màn hình với màu c (1 = trắng, 0 = đen).

```sig
clear()
```
Xóa buffer màn hình (thường tương đương fill(0))

```sig
text(s, x, y, color)
```
In ra chuỗi tại vị trí pixel thứ (x, y), `color`: `0` = đen, `1` = trắng.
```sig
text_wrap(s, x=0, y=0, c=1)
```
In chuỗi dài có tự ngắt dòng theo chiều rộng màn hình.
```sig
poweron()/poweroff()
```
Bật/tắt panel.
## Sample Code

```python
from oled import Oled
oled = Oled(driver="SH1106")
oled.clear()
oled.text("Hello, ESP32!", 0, 0)
oled.show()
```
