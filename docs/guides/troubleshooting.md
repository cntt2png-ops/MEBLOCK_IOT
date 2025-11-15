# Xử lý lỗi

- **OLED SH1106 lệch cột** → dùng `sh1106_offset=2`.
- **MQTT không nhận tin** → gọi `check_message()` đều trong loop.
- **DHT20 đọc fail** → kiểm tra dây I2C, địa chỉ `0x38`, nguồn ổn.
- **HC‑SR04 out of range** → kiểm tra dây, giảm khoảng cách.
