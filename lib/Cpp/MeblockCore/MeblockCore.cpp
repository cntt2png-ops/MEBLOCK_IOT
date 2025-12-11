// MeblockCore.cpp
#include "MeblockCore.h"
#include <Preferences.h>

extern "C" {
  #include "esp_system.h"
  #include "esp_ota_ops.h"
  #include "esp_partition.h"
  #include "esp_err.h"
}

// ===== DRD (Double Reset Detector) =====
static Preferences prefs;

static const uint32_t DRD_MAGIC   = 0xDEADBEEF;
static const uint32_t DRD_TIMEOUT = 8000;   // ms

static unsigned long drdStartTime = 0;
static bool drdArmed              = false;

// Buffer lệnh UART
static String uartCmdBuf;

// ================== CORE INTERNAL FUNCTIONS ==================

// Reset về FACTORY = app0 (OTA_0)
static void resetToFactory() {
  Serial.println("[MEBLOCK_CORE][FACTORY] Tìm partition FACTORY (app0 / OTA_0)...");

  const esp_partition_t* factory = esp_partition_find_first(
    ESP_PARTITION_TYPE_APP,
    ESP_PARTITION_SUBTYPE_APP_OTA_0,   // app0 = OTA_0 = FACTORY
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

// Khởi động DRD – gọi 1 lần trong setup()
static void setupDoubleResetDetector() {
  esp_reset_reason_t reason = esp_reset_reason();
  Serial.printf("[MEBLOCK_CORE][DRD] Reset reason = %d\n", (int)reason);

  // Chỉ dùng DRD cho POWERON / EXT reset (ấn nút reset, cúp nguồn)
  if (reason != ESP_RST_POWERON && reason != ESP_RST_EXT) {
    prefs.begin("drd", false);
    prefs.putUInt("flag", 0);   // clear flag nếu còn từ trước
    prefs.end();

    Serial.println("[MEBLOCK_CORE][DRD] Not POWERON/EXT → skip DRD & clear flag.");
    return;
  }

  prefs.begin("drd", false);
  uint32_t flag = prefs.getUInt("flag", 0);

  if (flag == DRD_MAGIC) {
    Serial.println("[MEBLOCK_CORE][DRD] Double reset detected → resetToFactory()");
    prefs.putUInt("flag", 0);   // clear flag
    prefs.end();

    resetToFactory();
    return;
  }

  Serial.println("[MEBLOCK_CORE][DRD] First reset, arm window for second reset...");
  prefs.putUInt("flag", DRD_MAGIC);
  prefs.end();

  drdStartTime = millis();
  drdArmed     = true;
}

// Gọi trong loop() để hết timeout thì hủy cờ DRD
static void handleDoubleResetDetector() {
  if (!drdArmed) return;

  if (millis() - drdStartTime > DRD_TIMEOUT) {
    prefs.begin("drd", false);
    prefs.putUInt("flag", 0);   // clear flag
    prefs.end();

    drdArmed = false;
    Serial.println("[MEBLOCK_CORE][DRD] Timeout, no double reset → normal run.");
  }
}

// Nhận lệnh qua UART: RESET_FACTORY / FACTORY
static void checkUartCommand() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '\r' || c == '\n') {
      uartCmdBuf.trim();

      // Nếu tool gửi literal "\n" hoặc "\r" (2 ký tự '\' và 'n') ở cuối → cắt bỏ
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

  setupDoubleResetDetector();
}

void meblock_core_loop() {
  handleDoubleResetDetector();
  checkUartCommand();
}
