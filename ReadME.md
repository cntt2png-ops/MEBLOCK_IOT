# MEBLOCK_IOT

Repo này là nơi lưu trữ và phát triển các thành phần cốt lõi cho hệ sinh thái **MEBLOCK** (website lập trình kéo thả & dạy học): **https://ide.meblock.cc/**

## Tổng quan

**MEBLOCK_IOT** tập trung vào phục vụ cho việc xây dựng, vận hành và mở rộng nền tảng MEBLOCK, bao gồm:

- **Core**: logic nền tảng, giao tiếp thiết bị, cấu trúc dữ liệu dự án/extension, tiện ích, ...
- **Firmware**: firmware cho các board/thiết bị, phục vụ kết nối, nạp/chạy chương trình, OTA, ...
- **Tools**: công cụ hỗ trợ phát triển: Build, Compile, Nạp, Run, Reset chương trình với MEBLOCK.
- **Extensions**: các extension cho IDE (block definitions, code generator, assets, metadata…).

Mục tiêu của repo là tạo một lưu trữ các dữ liệu kỹ thuật quan trọng phục vụ IDE hoạt động ổn định và việc mở rộng.

## Liên kết

- **MEBLOCK IDE**: https://ide.meblock.cc/
- **Repository**: https://github.com/cntt2png-ops/MEBLOCK_IOT

## Định hướng

Repo ưu tiên:
- **tái sử dụng** và **chuẩn hóa** giữa Nhiều loại Thiết bị ↔ Core ↔ Firmware ↔ Extensions
- QPhát triển rõ ràng nhằm ư dễ cộng tác.
- Hỗ trợ: dành cho cả **giáo dục** và **thiết bị** (device/robotics/IoT).

## Đóng góp

Nếu bạn muốn đóng góp (fix bug / cải tiến / thêm extension / cập nhật firmware):
- Tạo nhánh mới từ nhánh chính, commit rõ ràng theo từng mục.
- Mô tả ngắn gọn mục tiêu và phạm vi thay đổi trong Pull Request.

## License

MEBLOCK.
