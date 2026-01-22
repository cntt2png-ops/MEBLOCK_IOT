#include <Arduino.h>
#include <string>
#include <type_traits>
#include <utility>

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <Preferences.h>
#include <Update.h>

#include "IOT47_BLE_OTA.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_mac.h"

#if __has_include("esp_gatt_common_api.h")
  #include "esp_gatt_common_api.h"
  #define HAVE_ESP_GATT_MTU 1
#else
  #define HAVE_ESP_GATT_MTU 0
#endif

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

Preferences prefs;

// ===== Cấu hình MEBLOCK =====
static const char *MEBLOCK_NS     = "meblock";
static const char *KEY_NAME       = "name";
static const char *MEBLOCK_PREFIX = "MEBLOCK-";

String g_bleName;                 // tên BLE hiện tại
BLECharacteristic *pOtaCharacteristic = nullptr;

// Buffer lệnh UART (text)
String gSerialCmdBuf;

// ===== OTA RX buffering (tăng tốc + tránh nghẽn callback BLE) =====
// Lưu ý: iot47_ota_task nhận len kiểu uint8_t => tối đa 255 bytes mỗi "write".
static const uint16_t OTA_POOL_ITEM_SIZE = 255;   // tối đa dữ liệu enqueue mỗi gói
static const uint8_t  OTA_POOL_COUNT     = 32;    // tăng nếu muốn (tốn RAM)

static uint8_t  s_otaPool[OTA_POOL_COUNT][OTA_POOL_ITEM_SIZE];
static uint16_t s_otaLen[OTA_POOL_COUNT];

static QueueHandle_t s_otaFreeQ = nullptr;
static QueueHandle_t s_otaFillQ = nullptr;
static volatile bool s_otaQueuesReady = false;


// ===== Binary-safe read helper (works with both Bluedroid BLE & NimBLE wrappers) =====
template <typename T, typename = void>
struct has_getData : std::false_type {};
template <typename T>
struct has_getData<T, std::void_t<decltype(std::declval<T>().getData())>> : std::true_type {};

template <typename T, typename = void>
struct has_getLength : std::false_type {};
template <typename T>
struct has_getLength<T, std::void_t<decltype(std::declval<T>().getLength())>> : std::true_type {};

// Copy the most recent written value from characteristic into out[].
// IMPORTANT: avoids Arduino String truncation on binary data.
static size_t ble_read_value_bytes(BLECharacteristic *ch, uint8_t *out, size_t outMax) {
  if (!ch || !out || outMax == 0) return 0;

  // Arduino-ESP32 core 3.x BLE wrapper exposes binary-safe APIs:
  //   - getData()   -> pointer to value bytes
  //   - getLength() -> value length in bytes
  // Using getValue() (String) will truncate when the data contains 0x00 (OTA frames do).
  const uint8_t *data = (const uint8_t *)ch->getData();
  size_t len = (size_t)ch->getLength();

  if (!data || len == 0) return 0;
  if (len > outMax) len = outMax;
  memcpy(out, data, len);
  return len;
}


static bool isMeblockTextCommand(const String &cmd);
static bool ota_enqueue_from_bytes(const uint8_t *data, uint16_t dataLen, TickType_t waitTicks);
static void ota_worker_task(void *arg);


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
  // Giảm spam Serial để OTA nhanh và ổn định hơn
  static uint32_t lastMs = 0;
  static uint32_t lastCur = 0;

  uint32_t now = millis();
  bool timeHit = (now - lastMs) >= 250;      // log ~4 lần/giây
  bool stepHit = (cur - lastCur) >= 8192;    // hoặc mỗi 8KB
  bool done    = (total > 0 && cur >= total);

  if (timeHit || stepHit || done) {
    float pct = (total > 0) ? (100.0f * (float)cur / (float)total) : 0.0f;
	  Serial.printf("[OTA] %lu / %lu (%.1f%%)\n",
	              (unsigned long)cur,
	              (unsigned long)total,
	              pct);
    lastMs = now;
    lastCur = cur;
  }
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


// ===== Helpers for OTA buffering =====
static bool isMeblockTextCommand(const String &cmd) {
  if (cmd.length() == 0) return false;
  if (cmd.startsWith("NAME=")) return true;
  if (cmd.equalsIgnoreCase("NAME?")) return true;
  if (cmd.equalsIgnoreCase("BOOT_APP1")) return true;
  if (cmd.equalsIgnoreCase("APP1")) return true;
  return false;
}

static bool ota_enqueue_from_bytes(const uint8_t *data, uint16_t dataLen, TickType_t waitTicks) {
  if (!s_otaQueuesReady || !s_otaFreeQ || !s_otaFillQ) return false;

  uint8_t idx = 0xFF;
  if (xQueueReceive(s_otaFreeQ, &idx, waitTicks) != pdTRUE) {
    return false;
  }

  uint16_t n = dataLen;
  if (n > OTA_POOL_ITEM_SIZE) n = OTA_POOL_ITEM_SIZE;

  memcpy(s_otaPool[idx], data, n);
  s_otaLen[idx] = n;
  // Ensure 0-termination for text parsing inside iot47 (safe even for binary because len is passed)
  if (n < OTA_POOL_ITEM_SIZE) s_otaPool[idx][n] = 0;

  if (xQueueSend(s_otaFillQ, &idx, waitTicks) != pdTRUE) {
    // trả block về free nếu gửi vào fillQ thất bại
    xQueueSend(s_otaFreeQ, &idx, 0);
    return false;
  }

  return true;
}


// ---------------- OTA stream reassembly (handles BLE fragmentation) ----------------
// Protocol frame (binary): [pkt_hi][pkt_lo][len_hi][len_lo][payload...]
// Total frame length = 4 + payload_len, must be <= 255 (iot47_ota_task uses uint8_t len)

static uint8_t s_rxFrame[256];      // +1 for optional terminator
static uint16_t s_rxHave = 0;       // bytes currently in frame buffer
static uint16_t s_rxNeed = 0;       // total bytes needed for current frame (0 => header not complete yet)

static inline bool is_printable_ascii(uint8_t c) {
  return (c >= 0x20 && c <= 0x7E);
}

static void ota_stream_reset() {
  s_rxHave = 0;
  s_rxNeed = 0;
}

static void ota_stream_feed(const uint8_t *data, uint16_t len) {
  while (len > 0) {
    // If we're not currently building a binary frame and the chunk looks like text, pass through directly
    if (s_rxHave == 0 && s_rxNeed == 0) {
      // Typical control lines: "IOT47_BLE_OTA_BEGIN:xxxxx\r\n", "IOT47_BLE_OTA_END\r\n"
      // They start with printable ASCII, whereas binary frames usually start with 0x00 (packet high byte).
      if (is_printable_ascii(data[0])) {
        uint16_t n = (len > 255) ? 255 : len;
        memcpy(s_rxFrame, data, n);
        s_rxFrame[n] = 0; // safe terminator for text parsing
        (void)iot47_ota_task((uint8_t *)s_rxFrame, (uint8_t)n);
        // Consume all (we assume a single text command per write)
        return;
      }
    }

    // Binary frame reassembly
    if (s_rxNeed == 0) {
      // Need at least header (4 bytes)
      uint16_t needHdr = 4 - s_rxHave;
      uint16_t take = (len < needHdr) ? len : needHdr;
      memcpy(s_rxFrame + s_rxHave, data, take);
      s_rxHave += take;
      data += take;
      len  -= take;

      if (s_rxHave < 4) {
        continue; // still waiting for full header
      }

      uint16_t payloadLen = ((uint16_t)s_rxFrame[2] << 8) | (uint16_t)s_rxFrame[3];
      // iot47_ota_task len is uint8_t => total <= 255 => payload <= 251
      if (payloadLen > 251) {
        Serial.printf("[OTA] Bad payload len=%u -> drop/reset\n", (unsigned)payloadLen);
        ota_stream_reset();
        continue;
      }
      s_rxNeed = 4 + payloadLen;
    }

    // Copy payload bytes
    uint16_t remain = s_rxNeed - s_rxHave;
    uint16_t take = (len < remain) ? len : remain;
    memcpy(s_rxFrame + s_rxHave, data, take);
    s_rxHave += take;
    data += take;
    len  -= take;

    if (s_rxHave == s_rxNeed) {
      // Got full frame -> feed to OTA task
      s_rxFrame[s_rxHave] = 0; // safe terminator (doesn't change len passed)
      (void)iot47_ota_task((uint8_t *)s_rxFrame, (uint8_t)s_rxHave);
      ota_stream_reset();
    }
  }
}

static void ota_worker_task(void *arg) {
  (void)arg;
  uint8_t idx = 0;

  // Process OTA fragments from queue and reassemble into full frames
  for (;;) {
    if (xQueueReceive(s_otaFillQ, &idx, portMAX_DELAY) == pdTRUE) {
      uint16_t n = s_otaLen[idx];
      if (n > 0) {
        ota_stream_feed((const uint8_t *)s_otaPool[idx], n);
      }
      // return block to pool
      xQueueSend(s_otaFreeQ, &idx, 0);
      //taskYIELD();
      vTaskDelay(1);
    }
  }
}


// ===== BLE callbacks – dùng Arduino String (core ESP32 v3) =====
class MyCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) override {
    uint8_t rxBuf[OTA_POOL_ITEM_SIZE];
    size_t rxLen = ble_read_value_bytes(pCharacteristic, rxBuf, sizeof(rxBuf));
    if (rxLen == 0) return;

    // Text commands always start with a letter (A-Z / a-z).
    bool startsWithLetter =
        ((rxBuf[0] >= 'A' && rxBuf[0] <= 'Z') || (rxBuf[0] >= 'a' && rxBuf[0] <= 'z'));

    if (startsWithLetter) {
      String cmd;
      cmd.reserve(rxLen + 1);
      for (size_t i = 0; i < rxLen; i++) cmd += (char)rxBuf[i];
      cmd.trim();

      // Nếu là lệnh text MEBLOCK => xử lý ngay (ưu tiên)
      if (isMeblockTextCommand(cmd)) {
        Serial.print("[BLE] CMD = '");
        Serial.print(cmd);
        Serial.println("'");

        if (handleMeblockCommand(cmd)) {
          Serial.println("[BLE] Command processed.");
        } else {
          Serial.println("[BLE] Unknown or invalid command (ignored).");
        }
        return;
      }
      // Không phải command MEBLOCK => có thể là OTA control string (BEGIN/END/...)
      // -> rơi xuống xử lý OTA bên dưới (binary-safe)
    }

    // OTA packet / header => enqueue để tránh nghẽn callback BLE (tăng tốc + ổn định)
    if (s_otaQueuesReady) {
      if (!ota_enqueue_from_bytes(rxBuf, (uint16_t)rxLen, 0/*pdMS_TO_TICKS(100)*/)) {
        Serial.println("[OTA] RX buffer full -> FAIL:BUSY");
        if (pOtaCharacteristic) {
          pOtaCharacteristic->setValue("FAIL:BUSY");
          pOtaCharacteristic->notify();
        }
      }
      return;
    }

    // Fallback: xử lý trực tiếp (nếu queue chưa sẵn)
    uint8_t n = (rxLen > 255) ? 255 : (uint8_t)rxLen;
    int otaRes = iot47_ota_task((uint8_t *)rxBuf, n);
    if (otaRes != 0) {
      Serial.print("[OTA] iot47_ota_task handled packet, code=");
      Serial.println(otaRes);
      return;
    }

    // Không phải OTA và cũng không phải command => ignore
    Serial.println("[BLE] Unknown packet ignored.");
  }
};


// ===== SETUP & LOOP =====
#define SERVICE_UUID "55072829-bc9e-4c53-0003-74a6d4c78751"

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.println("=== MEBLOCK FACTORY + IOT47 BLE OTA ===");

  // 1) Tên BLE (MEBLOCK-... hoặc từ user)
  setupBleName();

  // 2) Init BLE
  Serial.print("[BLE] Device name: ");
  Serial.println(g_bleName);

  BLEDevice::init(g_bleName.c_str());

  // Tăng MTU (best-effort) để cải thiện throughput BLE
#if HAVE_ESP_GATT_MTU
  esp_err_t mtuErr = esp_ble_gatt_set_local_mtu(517);
  Serial.printf("[BLE] set_local_mtu(517) => %d\n", (int)mtuErr);
#else
  Serial.println("[BLE] esp_ble_gatt_set_local_mtu not available (core NimBLE). MTU will be negotiated by central.");
#endif
#if defined(ESP_PWR_LVL_P9)
  // Tăng TX power để ổn định kết nối (tuỳ board / môi trường)
  BLEDevice::setPower(ESP_PWR_LVL_P9);
#endif

  // Bật RX buffering cho OTA (giảm nghẽn callback BLE => tốc độ cao hơn)
  s_otaFreeQ = xQueueCreate(OTA_POOL_COUNT, sizeof(uint8_t));
  s_otaFillQ = xQueueCreate(OTA_POOL_COUNT, sizeof(uint8_t));
  if (s_otaFreeQ && s_otaFillQ) {
    for (uint8_t i = 0; i < OTA_POOL_COUNT; i++) {
      uint8_t idx = i;
      xQueueSend(s_otaFreeQ, &idx, 0);
    }
    s_otaQueuesReady = true;
    BaseType_t core = (portNUM_PROCESSORS > 1) ? 1 : 0;
    xTaskCreatePinnedToCore(ota_worker_task, "ota_worker", 8192, nullptr, 2, nullptr, core);

    Serial.printf("[OTA] RX buffering ON: pool=%u, item=%u\n", OTA_POOL_COUNT, OTA_POOL_ITEM_SIZE);
  } else {
    Serial.println("[OTA] RX buffering OFF (queue alloc failed)");
  }

  BLEServer *pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // 3) Characteristic OTA + command
  pOtaCharacteristic = pService->createCharacteristic(
      SERVICE_UUID,
      BLECharacteristic::PROPERTY_READ      |
      BLECharacteristic::PROPERTY_WRITE     |
      BLECharacteristic::PROPERTY_WRITE_NR  |
      BLECharacteristic::PROPERTY_NOTIFY    |
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
  pAdvertising->setMaxPreferred(0x0C);

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
