#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

class MeBlockMQTT {
public:
  MeBlockMQTT();
  using MsgCallback = void (*)(const String& value,
                               const String& action,
                               const String& channel,
                               const String& username);

  // ===== Blocks API =====.
  bool connect_wifi(const char* ssid, const char* password, uint32_t timeout_ms = 20000);

  bool connect_broker(const char* server, uint16_t port,
                      const char* user = "", const char* pass = "");

  bool connect_meblock(uint16_t port = 1883);

  // set username (dashboard)
  void connect_dashboard(const char* username);

  // NEW: set username + device_password
  void connect_dashboard(const char* username, const char* device_password);

  // NEW: set device password riêng
  void set_device_password(const char* device_password);

  // đăng ký handler cho (channel, action) và tự subscribe topic command tương ứng
  bool on_receive_message(const char* channel, const char* action, MsgCallback cb);

  // subscribe /command for a channel (for polling read_action_value / joystick without callback)
  bool subscribe_command(const char* channel);

// Read last received value by (channel, action)
String read_action_value(const String& channel, const String& action, const String& def = "");
long   read_action_value_int(const String& channel, const String& action, long def = 0);

// JOYSTICK_MOVE: value dạng "J7-94" (hoặc "F-94")
// - read_joystick_dir(): trả về CHUỖI trước dấu "-" (vd: "J7", "F")
// - read_joystick_dist(): trả về số sau dấu "-" (vd: 94)
String read_joystick_dir(const String& channel, const String& def = "");
int    read_joystick_dist(const String& channel, int def = -1);

  // publish value lên /data (tự đính kèm password nếu đã set)
  bool send_value(const char* channel, const String& value, bool retained = false);
  bool send_value(const char* channel, const char* value, bool retained = false);
  bool send_value(const char* channel, float value, bool retained = false);
  // FIX: tránh lỗi ambiguous khi truyền số thực kiểu double (ví dụ 25.6)
  bool send_value(const char* channel, double value, bool retained = false);
  bool send_value(const char* channel, int value, bool retained = false);
  bool send_value(const char* channel, bool value, bool retained = false);


  // send text/string (alias)
  bool send_string(const char* channel, const String& text, bool retained = false);
  bool send_string(const char* channel, const char* text, bool retained = false);

  // Compatibility alias (older code): send_text()
  bool send_text(const char* channel, const char* text, bool retained = false);

  // publish gói sensor lên /data (không phải format "value", nhưng vẫn kèm password)
  bool send_sensor_data(const char* channel, float temp, float hum, int bat = -1, long ts = -1);

  // loop + auto reconnect
  void check_message(bool auto_reconnect = true);

  bool is_connected();

private:
struct LastSlot {
  bool used = false;
  String channel;
  String action;
  String value;
  unsigned long ms = 0;
};

static const uint8_t MAX_LAST = 16;
LastSlot _last[MAX_LAST];

void storeLast_(const String& channel, const String& action, const String& value);
int  findLast_(const String& channel, const String& action) const;


  struct HandlerSlot {
    bool used = false;
    String channel;   // normalized: M1/M2...
    String action;
    MsgCallback cb = nullptr;
  };

  static constexpr uint8_t MAX_HANDLERS = 12;

  WiFiClient _net;
  PubSubClient _mqtt;

  // broker info để reconnect
  String _server;
  uint16_t _port = 1883;
  String _user;
  String _pass;

  // dashboard namespace
  String _username;

  // NEW: device password
  String _devicePassword;

  // handlers
  HandlerSlot _handlers[MAX_HANDLERS];
  String _subTopics[MAX_HANDLERS];

  // ===== internal =====
  static MeBlockMQTT* _self;
  static void mqttStaticCallback_(char* topic, byte* payload, unsigned int length);

  void handleMqttMessage_(const String& topic, const byte* payload, unsigned int length);

  bool ensureMqttConnected_();
  void resubscribeAll_();

  // NEW: normalize channel -> M1/M2...
  String normalizeChannel_(const char* channel) const;

  String topicData_(const char* channel) const;
  String topicCommand_(const char* channel) const;

  bool parseTopic_(const String& topic, String& outUsername, String& outChannel, String& outTail) const;
  String jsonValueToString_(JsonVariant v) const;

  String makeClientId_(const char* prefix = "MEBLOCK") const;
};

// global instance cho blockly gọi: mqtt.xxx()
extern MeBlockMQTT mqtt;
