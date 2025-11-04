# Xử lý lỗi

??? tip "Không thấy chữ trên SH1106"
    Thử `sh1106_offset=2` khi khởi tạo `Oled(...)` để canh khung 128 cột vào khu vực hiển thị.

??? tip "MQTT không nhận được tin"
    - Đảm bảo đã gọi `mqtt.check_message()` đều trong vòng lặp chính.
    - Kiểm tra wildcard trong `on_receive_message("demo/esp32/#", cb)`.

??? tip "DHT20 đọc fail"
    - Kiểm tra điện trở kéo‑lên (pull‑up) I2C và địa chỉ `0x38`.
    - Dây I2C ngắn, nguồn ổn định.
