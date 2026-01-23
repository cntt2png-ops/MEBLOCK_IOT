# MEBLOCK_IOT

Repo này là nơi lưu trữ và phát triển các thành phần cốt lõi cho hệ sinh thái **MEBLOCK IDE** (website lập trình kéo thả & dạy học): **https://ide.meblock.cc/**

## Tổng quan

**MEBLOCK_IOT** tập trung vào các hạng mục phục vụ cho việc xây dựng, vận hành và mở rộng nền tảng MEBLOCK, bao gồm:

- **Core**: logic nền tảng, chuẩn giao tiếp thiết bị, quy ước cấu trúc dự án/extension, tiện ích chung.
- **Firmware**: firmware cho các board/thiết bị (ví dụ ESP32/ESP32-S3/ESP32-C3…), phục vụ kết nối, nạp/chạy chương trình, OTA (nếu có), telemetry, v.v.
- **Tools**: công cụ hỗ trợ phát triển/đóng gói/kiểm thử/triển khai (CLI, scripts, build utilities…).
- **Extensions**: nguồn/khung phát triển extension cho IDE (block definitions, code generator, assets, metadata…).

Mục tiêu của repo là tạo một “nguồn sự thật” (single source of truth) cho các thành phần kỹ thuật quan trọng giúp IDE hoạt động ổn định và dễ mở rộng.

## Liên kết

- **MEBLOCK IDE**: https://ide.meblock.cc/
- **Repository**: https://github.com/cntt2png-ops/MEBLOCK_IOT

## Phạm vi & định hướng

Repo ưu tiên:
- Tính **tái sử dụng** (reusable modules) và **chuẩn hóa** (conventions) giữa Core ↔ Firmware ↔ Extensions.
- Quy trình phát triển rõ ràng để dễ cộng tác: tách phần chung, giảm phụ thuộc chéo, tài liệu hóa các thay đổi quan trọng.
- Hỗ trợ cả luồng dành cho **giáo dục** (dạy học) và luồng dành cho **thiết bị** (device/robotics/IoT).

## Đóng góp

Nếu bạn muốn đóng góp (fix bug / cải tiến / thêm extension / cập nhật firmware):
- Tạo nhánh mới từ nhánh chính, commit rõ ràng theo từng mục.
- Mô tả ngắn gọn mục tiêu và phạm vi thay đổi trong Pull Request.

## License

Chưa khai báo (sẽ bổ sung).
