#include "MeblockMQTT_dev.h"

// ===== Preset server MeBlock (dùng trong connect_meblock) =====
static const char* MEBLOCK_SERVER = "103.195.239.8";
static const char* MEBLOCK_USER   = "admin";
static const char* MEBLOCK_PASS   = "Leto@n1989";

MeBlockMQTT* MeBlockMQTT::_self = nullptr;

MeBlockMQTT::MeBlockMQTT() : _mqtt(_net) {}

bool MeBlockMQTT::connect_wifi(const char* ssid, const char* password, uint32_t timeout_ms) {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(200);
    if (millis() - t0 >= timeout_ms) return false;
  }
  return true;
}

bool MeBlockMQTT::connect_broker(const char* server, uint16_t port,
                                const char* user, const char* pass) {
  _server = server ? server : "";
  _port   = port;
  _user   = user ? user : "";
  _pass   = pass ? pass : "";

  _mqtt.setServer(_server.c_str(), _port);
  _self = this;
  _mqtt.setCallback(&MeBlockMQTT::mqttStaticCallback_);
  _mqtt.setBufferSize(768);

  return ensureMqttConnected_();
}

bool MeBlockMQTT::connect_meblock(uint16_t port) {
  return connect_broker(MEBLOCK_SERVER, port, MEBLOCK_USER, MEBLOCK_PASS);
}

void MeBlockMQTT::connect_dashboard(const char* username) {
  _username = username ? username : "";
}

void MeBlockMQTT::connect_dashboard(const char* username, const char* device_password) {
  _username = username ? username : "";
  _devicePassword = device_password ? device_password : "";
}

void MeBlockMQTT::set_device_password(const char* device_password) {
  _devicePassword = device_password ? device_password : "";
}

String MeBlockMQTT::normalizeChannel_(const char* channel) const {
  if (!channel) return "";
  String s = String(channel);
  s.trim();
  if (!s.length()) return "";

  auto isAllDigits = [&](const String& x) -> bool {
    if (!x.length()) return false;
    for (int i = 0; i < x.length(); i++) {
      if (!isDigit(x[i])) return false;
    }
    return true;
  };

  // "1" -> "M1"
  if (isAllDigits(s)) return "M" + s;

  // "m1"/"M1" -> "M1"
  // "v1"/"V1" -> "M1" (map legacy)
  if (s.length() >= 2) {
    char c0 = s[0];
    String rest = s.substring(1);
    if (isAllDigits(rest)) {
      if (c0 == 'm' || c0 == 'M') return "M" + rest;
      if (c0 == 'v' || c0 == 'V') return "M" + rest;
    }
  }

  // fallback giữ nguyên (để không phá trường hợp channel đặc biệt)
  return s;
}

bool MeBlockMQTT::on_receive_message(const char* channel, const char* action, MsgCallback cb) {
  if (!_username.length()) return false;
  if (!channel || !action || !cb) return false;

  int idx = -1;
  for (int i = 0; i < MAX_HANDLERS; i++) {
    if (!_handlers[i].used) { idx = i; break; }
  }
  if (idx < 0) return false;

  String ch = normalizeChannel_(channel);
  if (!ch.length()) return false;

  _handlers[idx].used = true;
  _handlers[idx].channel = ch;       // store normalized
  _handlers[idx].action  = action;
  _handlers[idx].cb      = cb;

  _subTopics[idx] = topicCommand_(ch.c_str());

  if (!ensureMqttConnected_()) return false;
  return _mqtt.subscribe(_subTopics[idx].c_str());
}

bool MeBlockMQTT::send_value(const char* channel, const String& value, bool retained) {
  if (!_username.length() || !channel) return false;
  if (!ensureMqttConnected_()) return false;

  String ch = normalizeChannel_(channel);
  if (!ch.length()) return false;

  StaticJsonDocument<256> doc;
  doc["value"] = value;
  if (_devicePassword.length()) doc["password"] = _devicePassword;

  char buf[384];
  size_t n = serializeJson(doc, buf, sizeof(buf));
  if (n == 0) return false;

  String topic = topicData_(ch.c_str());
  return _mqtt.publish(topic.c_str(), buf, retained);
}

bool MeBlockMQTT::send_value(const char* channel, float value, bool retained) {
  if (!_username.length() || !channel) return false;
  if (!ensureMqttConnected_()) return false;

  String ch = normalizeChannel_(channel);
  if (!ch.length()) return false;

  StaticJsonDocument<256> doc;
  doc["value"] = value;
  if (_devicePassword.length()) doc["password"] = _devicePassword;

  char buf[384];
  size_t n = serializeJson(doc, buf, sizeof(buf));
  if (n == 0) return false;

  String topic = topicData_(ch.c_str());
  return _mqtt.publish(topic.c_str(), buf, retained);
}

// FIX: overload cho double để tránh ambiguous (double có thể match float/int/bool)
bool MeBlockMQTT::send_value(const char* channel, double value, bool retained) {
  if (!_username.length() || !channel) return false;
  if (!ensureMqttConnected_()) return false;

  String ch = normalizeChannel_(channel);
  if (!ch.length()) return false;

  StaticJsonDocument<256> doc;
  doc["value"] = value;
  if (_devicePassword.length()) doc["password"] = _devicePassword;

  char buf[384];
  size_t n = serializeJson(doc, buf, sizeof(buf));
  if (n == 0) return false;

  String topic = topicData_(ch.c_str());
  return _mqtt.publish(topic.c_str(), buf, retained);
}

bool MeBlockMQTT::send_value(const char* channel, int value, bool retained) {
  if (!_username.length() || !channel) return false;
  if (!ensureMqttConnected_()) return false;

  String ch = normalizeChannel_(channel);
  if (!ch.length()) return false;

  StaticJsonDocument<256> doc;
  doc["value"] = value;
  if (_devicePassword.length()) doc["password"] = _devicePassword;

  char buf[384];
  size_t n = serializeJson(doc, buf, sizeof(buf));
  if (n == 0) return false;

  String topic = topicData_(ch.c_str());
  return _mqtt.publish(topic.c_str(), buf, retained);
}

bool MeBlockMQTT::send_value(const char* channel, bool value, bool retained) {
  if (!_username.length() || !channel) return false;
  if (!ensureMqttConnected_()) return false;

  String ch = normalizeChannel_(channel);
  if (!ch.length()) return false;

  StaticJsonDocument<256> doc;
  doc["value"] = value;
  if (_devicePassword.length()) doc["password"] = _devicePassword;

  char buf[384];
  size_t n = serializeJson(doc, buf, sizeof(buf));
  if (n == 0) return false;

  String topic = topicData_(ch.c_str());
  return _mqtt.publish(topic.c_str(), buf, retained);
}

bool MeBlockMQTT::send_sensor_data(const char* channel, float temp, float hum, int bat, long ts) {
  if (!_username.length() || !channel) return false;
  if (!ensureMqttConnected_()) return false;

  String ch = normalizeChannel_(channel);
  if (!ch.length()) return false;

  StaticJsonDocument<384> doc;
  doc["temp"] = temp;
  doc["hum"]  = hum;
  if (bat >= 0) doc["bat"] = bat;
  if (ts  >= 0) doc["ts"]  = ts;
  if (_devicePassword.length()) doc["password"] = _devicePassword;

  char buf[512];
  size_t n = serializeJson(doc, buf, sizeof(buf));
  if (n == 0) return false;

  String topic = topicData_(ch.c_str());
  return _mqtt.publish(topic.c_str(), buf, false);
}

void MeBlockMQTT::check_message(bool auto_reconnect) {
  if (auto_reconnect) ensureMqttConnected_();
  _mqtt.loop();
}

bool MeBlockMQTT::is_connected() {
  return _mqtt.connected();
}

// ===== internal =====

void MeBlockMQTT::mqttStaticCallback_(char* topic, byte* payload, unsigned int length) {
  if (_self) _self->handleMqttMessage_(String(topic), payload, length);
}

void MeBlockMQTT::handleMqttMessage_(const String& topic, const byte* payload, unsigned int length) {
  // chỉ xử lý command
  String tUser, tChannel, tTail;
  if (!parseTopic_(topic, tUser, tChannel, tTail)) return;
  if (tTail != "command") return;

  // normalize channel từ topic (V1 -> M1, "1" -> M1)
  String chNorm = normalizeChannel_(tChannel.c_str());

  StaticJsonDocument<384> doc;
  DeserializationError err = deserializeJson(doc, payload, length);
  if (err) return;

  // ===== PASSWORD CHECK (NEW) =====
  if (_devicePassword.length()) {
    JsonVariant p = doc["password"];
    String recvPass = jsonValueToString_(p);
    if (!recvPass.length() || recvPass != _devicePassword) {
      // sai pass => bỏ qua (giống code bạn)
      return;
    }
  }

  String action = doc["action"] | "";
  JsonVariant v = doc["value"];
  String valueStr = jsonValueToString_(v);

  // dispatch theo (channel, action)
  for (int i = 0; i < MAX_HANDLERS; i++) {
    if (!_handlers[i].used || !_handlers[i].cb) continue;
    if (_handlers[i].channel == chNorm && _handlers[i].action == action) {
      _handlers[i].cb(valueStr, action, chNorm, tUser);
    }
  }
}

bool MeBlockMQTT::ensureMqttConnected_() {
  if (_mqtt.connected()) return true;
  if (!_server.length()) return false;

  String cid = makeClientId_("MEBLOCK");
  bool ok;
  if (_user.length()) ok = _mqtt.connect(cid.c_str(), _user.c_str(), _pass.c_str());
  else ok = _mqtt.connect(cid.c_str());

  if (ok) resubscribeAll_();
  return ok;
}

void MeBlockMQTT::resubscribeAll_() {
  for (int i = 0; i < MAX_HANDLERS; i++) {
    if (_handlers[i].used && _subTopics[i].length()) {
      _mqtt.subscribe(_subTopics[i].c_str());
    }
  }
}

String MeBlockMQTT::topicData_(const char* channel) const {
  String ch = normalizeChannel_(channel);
  String t = "meblock/";
  t += _username;
  t += "/";
  t += ch;
  t += "/data";
  return t;
}

String MeBlockMQTT::topicCommand_(const char* channel) const {
  String ch = normalizeChannel_(channel);
  String t = "meblock/";
  t += _username;
  t += "/";
  t += ch;
  t += "/command";
  return t;
}

bool MeBlockMQTT::parseTopic_(const String& topic, String& outUsername, String& outChannel, String& outTail) const {
  int p1 = topic.indexOf('/');
  if (p1 < 0) return false;
  String head = topic.substring(0, p1);
  if (head != "meblock") return false;

  int p2 = topic.indexOf('/', p1 + 1);
  if (p2 < 0) return false;
  int p3 = topic.indexOf('/', p2 + 1);
  if (p3 < 0) return false;

  outUsername = topic.substring(p1 + 1, p2);
  outChannel  = topic.substring(p2 + 1, p3);
  outTail     = topic.substring(p3 + 1);
  return true;
}

String MeBlockMQTT::jsonValueToString_(JsonVariant v) const {
  if (v.is<const char*>()) return String(v.as<const char*>());
  if (v.is<String>())      return v.as<String>();
  if (v.is<bool>())        return v.as<bool>() ? "true" : "false";
  if (v.is<long>())        return String(v.as<long>());
  if (v.is<double>())      return String(v.as<double>());

  String out;
  serializeJson(v, out);
  return out;
}

String MeBlockMQTT::makeClientId_(const char* prefix) const {
  uint64_t mac = ESP.getEfuseMac();
  char buf[40];

  unsigned int hi = (unsigned int)((mac >> 32) & 0xFFFF);
  unsigned long lo = (unsigned long)(mac & 0xFFFFFFFFUL);

  snprintf(buf, sizeof(buf), "%s-%04X%08lX", prefix ? prefix : "MEBLOCK", hi, lo);
  return String(buf);
}

// Global instance cho Blockly/Sketch dùng trực tiếp: mqtt.xxx()
// Lưu ý: KHÔNG khai báo lại `MeBlockMQTT mqtt;` ở sketch để tránh multiple definition.
MeBlockMQTT mqtt;
