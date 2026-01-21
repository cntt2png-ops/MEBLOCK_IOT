// MeblockCore.h
#pragma once
#include <Arduino.h>

/// Khởi tạo core: Serial + Double Reset Detector (DRD)
/// Mặc định dùng baud 115200, có thể đổi nếu cần.
void meblock_core_setup(uint32_t serialBaud = 115200);

/// Hàm loop của core: xử lý DRD + lệnh UART RESET_FACTORY
/// Gọi mỗi vòng loop() trước khi chạy code Blockly.
void meblock_core_loop();
