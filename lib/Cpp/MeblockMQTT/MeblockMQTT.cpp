#include "MeblockMQTT.h"

// ===== Preset server MeBlock (dùng trong connect_meblock) =====
static const char* MEBLOCK_SERVER = "103.195.239.8";
static const char* MEBLOCK_USER   = "admin";
static const char* MEBLOCK_PASS   = "Leto@n1989";

MeBlockMQTT* MeBlockMQTT::_self = nullptr;
MeBlockMQTT mqtt;

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
  // dùng preset => hết warning "defined but not used"
  return connect_broker(MEBLOCK_SERVER, port, MEBLOCK_USER, MEBLOCK_PASS);
}

void MeBlockMQTT::connect_dashboard(const char* username) {
  _username = username ? username : "";
}

bool MeBlockMQTT::on_receive_message(const char* channel, const char* action, MsgCallback cb) {
  if (!_username.length()) return false;
  if (!channel || !action || !cb) return false;

  // tìm slot trống
  int idx = -1;
  for (int i = 0; i < MAX_HANDLERS; i++) {
    if (!_handlers[i].used) { idx = i; break; }
  }
  if (idx < 0) return false;

  _handlers[idx].used = true;
  _handlers[idx].channel = channel;
  _handlers[idx].action  = action;
  _handlers[idx].cb      = cb;

  // subscribe topic command
  _subTopics[idx] = topicCommand_(channel);
  if (!ensureMqttConnected_()) return false;

  return _mqtt.subscribe(_subTopics[idx].c_str());
}

bool MeBlockMQTT::send_value(const char* channel, const String& value, bool retained) {
  if (!_username.length() || !channel) return false;
  if (!ensureMqttConnected_()) return false;

  JsonDocument doc;
  doc["value"] = value;

  char buf[256];
  size_t n = serializeJson(doc, buf, sizeof(buf));
  if (n == 0) return false;

  String topic = topicData_(channel);
  return _mqtt.publish(topic.c_str(), buf, retained);
}

bool MeBlockMQTT::send_value(const char* channel, float value, bool retained) {
  if (!_username.length() || !channel) return false;
  if (!ensureMqttConnected_()) return false;

  JsonDocument doc;
  doc["value"] = value;

  char buf[256];
  size_t n = serializeJson(doc, buf, sizeof(buf));
  if (n == 0) return false;

  String topic = topicData_(channel);
  return _mqtt.publish(topic.c_str(), buf, retained);
}

bool MeBlockMQTT::send_value(const char* channel, int value, bool retained) {
  if (!_username.length() || !channel) return false;
  if (!ensureMqttConnected_()) return false;

  JsonDocument doc;
  doc["value"] = value;

  char buf[256];
  size_t n = serializeJson(doc, buf, sizeof(buf));
  if (n == 0) return false;

  String topic = topicData_(channel);
  return _mqtt.publish(topic.c_str(), buf, retained);
}

bool MeBlockMQTT::send_value(const char* channel, bool value, bool retained) {
  if (!_username.length() || !channel) return false;
  if (!ensureMqttConnected_()) return false;

  JsonDocument doc;
  doc["value"] = value;

  char buf[256];
  size_t n = serializeJson(doc, buf, sizeof(buf));
  if (n == 0) return false;

  String topic = topicData_(channel);
  return _mqtt.publish(topic.c_str(), buf, retained);
}

bool MeBlockMQTT::send_sensor_data(const char* channel, float temp, float hum, int bat, long ts) {
  if (!_username.length() || !channel) return false;
  if (!ensureMqttConnected_()) return false;

  JsonDocument doc;
  doc["temp"] = temp;
  doc["hum"]  = hum;
  if (bat >= 0) doc["bat"] = bat;
  if (ts  >= 0) doc["ts"]  = ts;

  char buf[384];
  size_t n = serializeJson(doc, buf, sizeof(buf));
  if (n == 0) return false;

  String topic = topicData_(channel);
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

  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, payload, length);
  if (err) return;

  String action = doc["action"] | "";
  JsonVariant v = doc["value"];

  String valueStr = jsonValueToString_(v);

  // dispatch theo (channel, action)
  for (int i = 0; i < MAX_HANDLERS; i++) {
    if (!_handlers[i].used || !_handlers[i].cb) continue;
    if (_handlers[i].channel == tChannel && _handlers[i].action == action) {
      _handlers[i].cb(valueStr, action, tChannel, tUser);
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
  // meblock/{username}/{channel}/data
  String t = "meblock/";
  t += _username;
  t += "/";
  t += channel;
  t += "/data";
  return t;
}

String MeBlockMQTT::topicCommand_(const char* channel) const {
  // meblock/{username}/{channel}/command
  String t = "meblock/";
  t += _username;
  t += "/";
  t += channel;
  t += "/command";
  return t;
}

bool MeBlockMQTT::parseTopic_(const String& topic, String& outUsername, String& outChannel, String& outTail) const {
  // expect: meblock/{username}/{channel}/{tail}
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

  // fallback: serialize value node
  String out;
  serializeJson(v, out);
  return out;
}

String MeBlockMQTT::makeClientId_(const char* prefix) const {
  uint64_t mac = ESP.getEfuseMac();
  char buf[40];

  // FIX warning format: dùng %08lX cho unsigned long
  unsigned int hi = (unsigned int)((mac >> 32) & 0xFFFF);
  unsigned long lo = (unsigned long)(mac & 0xFFFFFFFFUL);

  snprintf(buf, sizeof(buf), "%s-%04X%08lX", prefix ? prefix : "MEBLOCK", hi, lo);
  return String(buf);
}
