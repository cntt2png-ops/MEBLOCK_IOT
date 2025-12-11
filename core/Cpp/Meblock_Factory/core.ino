#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <Preferences.h>
#include <Update.h>

#include "IOT47_BLE_OTA.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_mac.h"

Preferences prefs;

// ===== Cấu hình MEBLOCK =====
static const char *MEBLOCK_NS     = "meblock";
static const char *KEY_NAME       = "name";
static const char *MEBLOCK_PREFIX = "MEBLOCK-";

String g_bleName;                 // tên BLE hiện tại
BLECharacteristic *pOtaCharacteristic = nullptr;

// Buffer lệnh UART (text)
String gSerialCmdBuf;

// ===== Forward declarations =====
String generateDefaultNameFromMac();
String normalizeNameWithPrefix(const String &raw);
void saveDeviceName(const String &name);
bool handleMeblockCommand(const String &cmd);
void cmdBootApp1();
void setupBleName();

// ===== OTA callbacks =====
void ota_begin_cb(uint32_t cur, uint32_t total) {
  Serial.println("[OTA] Begin OTA...");
}

void ota_process_cb(uint32_t cur, uint32_t total) {
  Serial.printf("[OTA] %lu / %lu\n",
                (unsigned long)cur,
                (unsigned long)total);
}

void ota_end_cb(uint32_t cur, uint32_t total) {
  Serial.println("[OTA] Download done, restarting to new firmware...");
}

void ota_error_cb(uint32_t cur, uint32_t total) {
  Serial.println("[OTA] Download error!");
}

// ===== Sinh tên default theo MAC: MEBLOCK-XXYYZZ =====
String generateDefaultNameFromMac() {
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_WIFI_STA);  // MAC WiFi STA

  char suffix[7];
  snprintf(suffix, sizeof(suffix), "%02X%02X%02X", mac[3], mac[4], mac[5]);
  return String(MEBLOCK_PREFIX) + suffix;
}

// Đảm bảo có prefix MEBLOCK-
String normalizeNameWithPrefix(const String &raw) {
  String trimmed = raw;
  trimmed.trim();
  if (trimmed.length() == 0) {
    return generateDefaultNameFromMac();
  }
  if (trimmed.startsWith(MEBLOCK_PREFIX)) {
    return trimmed;
  }
  return String(MEBLOCK_PREFIX) + trimmed;
}

// Lưu vào NVS + cập nhật g_bleName
void saveDeviceName(const String &name) {
  String norm = normalizeNameWithPrefix(name);
  prefs.putString(KEY_NAME, norm);
  g_bleName = norm;

  Serial.print("[MEBLOCK] Saved device name: ");
  Serial.println(g_bleName);
}

// Lệnh BOOT_APP1 / APP1 → nhảy sang OTA_0 (app1)
void cmdBootApp1() {
  Serial.println("[MEBLOCK] Switching boot partition to app1 (OTA_1)...");

  const esp_partition_t *app1 = esp_partition_find_first(
      ESP_PARTITION_TYPE_APP,
      ESP_PARTITION_SUBTYPE_APP_OTA_1,   // <-- dùng OTA_1, giống core cũ
      nullptr
  );

  if (!app1) {
    Serial.println("[MEBLOCK] ERROR: app1 (OTA_1) partition not found!");
    return;
  }

  Serial.printf(
      "[MEBLOCK] Set boot to %s (addr=0x%08lX, size=0x%lX)\n",
      app1->label,
      (unsigned long)app1->address,
      (unsigned long)app1->size
  );

  esp_err_t err = esp_ota_set_boot_partition(app1);
  if (err == ESP_OK) {
    Serial.println("[MEBLOCK] Restarting to app1...");
    delay(200);
    ESP.restart();
  } else {
    Serial.printf("[MEBLOCK] ERROR: esp_ota_set_boot_partition failed, err=%d\n", (int)err);
  }
}


// Xử lý lệnh text của MEBLOCK (dùng chung BLE + UART)
bool handleMeblockCommand(const String &cmdIn) {
  String cmd = cmdIn;
  cmd.trim();

  // Hỏi tên hiện tại
  if (cmd.equalsIgnoreCase("NAME?")) {
    Serial.print("[MEBLOCK] NAME = ");
    Serial.println(g_bleName);
    return true;
  }

  // Boot sang app1
  if (cmd.equalsIgnoreCase("BOOT_APP1") || cmd.equalsIgnoreCase("APP1")) {
    cmdBootApp1();
    return true;
  }

  // Đổi tên: NAME=<suffix> hoặc NAME=MEBLOCK-XXXX
  if (cmd.startsWith("NAME=")) {
    String value = cmd.substring(5);
    value.trim();

    if (value.length() == 0 || value.equalsIgnoreCase("RESET")) {
      String defName = generateDefaultNameFromMac();
      saveDeviceName(defName);
    } else {
      saveDeviceName(value);
    }

    Serial.println("[MEBLOCK] Name changed, rebooting to apply BLE name...");
    delay(200);
    ESP.restart();
    return true;
  }

  return false;
}

// Load / init tên BLE từ NVS
void setupBleName() {
  prefs.begin(MEBLOCK_NS, false);

  String saved = prefs.getString(KEY_NAME, "");
  if (saved.length() == 0) {
    g_bleName = generateDefaultNameFromMac();
    prefs.putString(KEY_NAME, g_bleName);
    Serial.print("[MEBLOCK] No stored name, using default from MAC: ");
    Serial.println(g_bleName);
  } else {
    g_bleName = normalizeNameWithPrefix(saved);
    if (g_bleName != saved) {
      prefs.putString(KEY_NAME, g_bleName);
    }
    Serial.print("[MEBLOCK] Loaded stored name: ");
    Serial.println(g_bleName);
  }
}

// ===== BLE callbacks – dùng Arduino String (core ESP32 v3) =====
class MyCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) override {
    // Core ESP32 v3: getValue() trả về Arduino String
    String rxStr = pCharacteristic->getValue();
    if (rxStr.length() == 0) {
      return;
    }

    // 1) IOT47 OTA (binary)
    int otaRes = iot47_ota_task(
        (uint8_t *)rxStr.c_str(),
        (uint8_t)rxStr.length()
    );
    if (otaRes != 0) {
      Serial.print("[OTA] iot47_ota_task handled packet, code=");
      Serial.println(otaRes);
      return;
    }

    // 2) Lệnh MEBLOCK (text)
    String cmd = rxStr;
    cmd.trim();

    Serial.print("[BLE] CMD = '");
    Serial.print(cmd);
    Serial.println("'");

    if (handleMeblockCommand(cmd)) {
      Serial.println("[BLE] Command processed.");
    } else {
      Serial.println("[BLE] Unknown or invalid command (ignored).");
    }
  }
};

// ===== SETUP & LOOP =====
#define SERVICE_UUID "55072829-bc9e-4c53-0003-74a6d4c78751"

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.println("=== MEBLOCK FACTORY + IOT47 BLE OTA (ESP32-S3) ===");

  // 1) Tên BLE (MEBLOCK-... hoặc từ user)
  setupBleName();

  // 2) Init BLE
  Serial.print("[BLE] Device name: ");
  Serial.println(g_bleName);

  BLEDevice::init(g_bleName.c_str());
  BLEServer *pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // 3) Characteristic OTA + command
  pOtaCharacteristic = pService->createCharacteristic(
      SERVICE_UUID,
      BLECharacteristic::PROPERTY_READ   |
      BLECharacteristic::PROPERTY_WRITE  |
      BLECharacteristic::PROPERTY_NOTIFY |
      BLECharacteristic::PROPERTY_INDICATE
  );

  pOtaCharacteristic->setCallbacks(new MyCallbacks());
  pOtaCharacteristic->setValue("MEBLOCK FACTORY READY");

  // Đăng ký cho IOT47 BLE OTA
  iot47_ble_ota_begin(pOtaCharacteristic);
  iot47_ble_ota_set_begin_callback(ota_begin_cb);
  iot47_ble_ota_set_proces_callback(ota_process_cb);
  iot47_ble_ota_set_end_callback(ota_end_cb);
  iot47_ble_ota_set_error_callback(ota_error_cb);
  iot47_stop_ota();  // đảm bảo state = OTA_BEGIN khi khởi động

  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMaxPreferred(0x12);

  BLEDevice::startAdvertising();

  Serial.println("[BLE] Advertising started.");
}

void loop() {
  // Xử lý lệnh UART (text, kết thúc '\n')
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n') {
      String cmd = gSerialCmdBuf;
      gSerialCmdBuf = "";
      cmd.trim();

      if (cmd.length() > 0) {
        Serial.print("[UART] CMD = '");
        Serial.print(cmd);
        Serial.println("'");

        if (!handleMeblockCommand(cmd)) {
          Serial.println("[UART] Unknown command.");
        }
      }
    } else if (c != '\r') {
      gSerialCmdBuf += c;
    }
  }

  delay(10);
}
