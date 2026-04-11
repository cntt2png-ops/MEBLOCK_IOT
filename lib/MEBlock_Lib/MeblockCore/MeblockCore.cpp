// MeblockCore.cpp
#include "MeblockCore.h"

extern "C" {
  #include "esp_system.h"
  #include "esp_ota_ops.h"
  #include "esp_partition.h"
  #include "esp_err.h"
}

// ===== BOOT button detector =====
static const int BOOT_BUTTON_PIN = 0;
static const uint32_t DOUBLE_BOOT_WINDOW_MS = 3000;
static const uint32_t DEBOUNCE_MS = 60;

// Buffer lệnh UART
static String uartCmdBuf;

// ================== CORE INTERNAL FUNCTIONS ==================

// Reset về FACTORY = app0 (OTA_0)
static void resetToFactory() {
  Serial.println("[MEBLOCK_CORE][FACTORY] Tìm partition FACTORY (app0 / OTA_0)...");

  const esp_partition_t* factory = esp_partition_find_first(
    ESP_PARTITION_TYPE_APP,
    ESP_PARTITION_SUBTYPE_APP_OTA_0,
    nullptr
  );

  if (!factory) {
    Serial.println("[MEBLOCK_CORE][FACTORY] ERROR: Không tìm thấy partition app0 (OTA_0)!");
    return;
  }

  Serial.printf("[MEBLOCK_CORE][FACTORY] Found app0: label=%s addr=0x%X size=0x%X\n",
                factory->label, factory->address, factory->size);

  esp_err_t err = esp_ota_set_boot_partition(factory);
  if (err != ESP_OK) {
    Serial.print("[MEBLOCK_CORE][FACTORY] ERROR: esp_ota_set_boot_partition failed, err=");
    Serial.println((int)err);
    return;
  }

  Serial.println("[MEBLOCK_CORE][FACTORY] Đã set boot về FACTORY (app0). Restart...");
  delay(200);
  esp_restart();
}

// Kiểm tra nhấn 2 lần nút BOOT (IO0) trong lúc khởi động
static bool checkDoubleBootAtStartup() {
  pinMode(BOOT_BUTTON_PIN, INPUT_PULLUP);

  Serial.println("[MEBLOCK_CORE][BOOT] Press BOOT twice to enter FACTORY...");

  int pressCount = 0;
  int lastState = digitalRead(BOOT_BUTTON_PIN);
  unsigned long startMs = millis();
  unsigned long lastChangeMs = startMs;

  while (millis() - startMs < DOUBLE_BOOT_WINDOW_MS) {
    int state = digitalRead(BOOT_BUTTON_PIN);
    unsigned long now = millis();

    if (state != lastState && (now - lastChangeMs) >= DEBOUNCE_MS) {
      lastState = state;
      lastChangeMs = now;

      // BOOT là active-low
      if (state == LOW) {
        pressCount++;
        Serial.printf("[MEBLOCK_CORE][BOOT] Press %d\n", pressCount);

        if (pressCount >= 2) {
          return true;
        }
      }
    }

    delay(10);
  }

  return false;
}

// Nhận lệnh qua UART: RESET_FACTORY / FACTORY
static void checkUartCommand() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '\r' || c == '\n') {
      uartCmdBuf.trim();

      // Nếu tool gửi literal "\\n" hoặc "\\r" (2 ký tự '\\' và 'n') ở cuối → cắt bỏ
      if (uartCmdBuf.endsWith("\\n") || uartCmdBuf.endsWith("\\r")) {
        uartCmdBuf.remove(uartCmdBuf.length() - 2);
        uartCmdBuf.trim();
      }

      if (uartCmdBuf.length() > 0) {
        Serial.print("[MEBLOCK_CORE][UART] CMD = '");
        Serial.print(uartCmdBuf);
        Serial.println("'");

        if (uartCmdBuf.equalsIgnoreCase("RESET_FACTORY") ||
            uartCmdBuf.equalsIgnoreCase("FACTORY")) {
          Serial.println("[MEBLOCK_CORE][UART] Nhận lệnh RESET_FACTORY → resetToFactory()");
          resetToFactory();
        }
      }

      uartCmdBuf = "";
    } else {
      if (uartCmdBuf.length() < 64) {
        uartCmdBuf += c;
      } else {
        // Quá dài thì reset buffer cho an toàn
        uartCmdBuf = "";
      }
    }
  }
}

// ================== PUBLIC API ==================

void meblock_core_setup(uint32_t serialBaud) {
  Serial.begin(serialBaud);
  delay(200);
  Serial.println("\n[MEBLOCK_CORE] Init...");
  OnboardRGB.begin();

  if (checkDoubleBootAtStartup()) {
    Serial.println("[MEBLOCK_CORE][BOOT] Double BOOT detected → resetToFactory()");
    resetToFactory();
  }
}

void meblock_core_loop() {
  checkUartCommand();
}
