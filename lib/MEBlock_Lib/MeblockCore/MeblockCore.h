// MeblockCore.h
#pragma once
#include <Arduino.h>
#include "MeblockOnboard.h"

/// Khởi tạo core: Serial + OnboardRGB + kiểm tra BOOT IO0 nhấn 2 lần lúc startup.
/// Nếu nhấn 2 lần trong cửa sổ khởi động, core sẽ set boot về FACTORY/app0 (OTA_0).
void meblock_core_setup(uint32_t serialBaud = 115200);

/// Hàm loop của core: chỉ xử lý lệnh UART RESET_FACTORY / FACTORY.
/// Gọi mỗi vòng loop() trước khi chạy code Blockly.
void meblock_core_loop();
