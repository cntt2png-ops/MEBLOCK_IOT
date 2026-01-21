#include "MeblockGamepadBLE.h"
#include <math.h>
#include <string.h>

bool MeblockGamepadBLE::begin(const char* name, int deadzone) {
  _deadzone = deadzone;

#if MEBLOCK_USE_NIMBLE
  NimBLEDevice::init(name);

  _server = NimBLEDevice::createServer();
  _server->setCallbacks(new _ServerCB(this));

  _svc = _server->createService(NUS_SERVICE);

  _tx = _svc->createCharacteristic(NUS_TX_CHAR, NIMBLE_PROPERTY::NOTIFY);

  _rx = _svc->createCharacteristic(
    NUS_RX_CHAR,
    NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR
  );
  _rx->setCallbacks(new _RxCB(this));

  _svc->start();

  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  adv->addServiceUUID(NUS_SERVICE);
  adv->setScanResponse(true);
  adv->start();

#else
  BLEDevice::init(name);

  _server = BLEDevice::createServer();
  _server->setCallbacks(new _ServerCB(this));

  _svc = _server->createService(NUS_SERVICE);

  _tx = _svc->createCharacteristic(
    NUS_TX_CHAR,
    BLECharacteristic::PROPERTY_NOTIFY
  );
  _tx->addDescriptor(new BLE2902());

  _rx = _svc->createCharacteristic(
    NUS_RX_CHAR,
    BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR
  );
  _rx->setCallbacks(new _RxCB(this));

  _svc->start();

  BLEAdvertising* adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(NUS_SERVICE);
  adv->setScanResponse(true);
  adv->start();

#endif

  _sendNotify("READY\n");
  return true;
}

void MeblockGamepadBLE::end() {
  // Safe end(): do not call deinit() to avoid core-dependent compile issues.
  _server = nullptr;
  _svc = nullptr;
  _rx = nullptr;
  _tx = nullptr;
  _connected = false;
}

bool MeblockGamepadBLE::waitConnected(uint32_t timeoutMs, uint32_t pollMs) {
  uint32_t t0 = millis();
  while (true) {
    if (isConnected()) return true;
    if (timeoutMs && (millis() - t0 > timeoutMs)) return false;
    delay(pollMs);
  }
}

uint32_t MeblockGamepadBLE::ageMs() const {
  if (_lastSnapshotMs == 0) return 0xFFFFFFFFu;
  return millis() - _lastSnapshotMs;
}

void MeblockGamepadBLE::_sendNotify(const char* s) {
  if (!_tx) return;
  _tx->setValue((uint8_t*)s, strlen(s));
  _tx->notify();
}

void MeblockGamepadBLE::_pushBytes(const uint8_t* data, size_t len) {
  _lock();
  for (size_t i = 0; i < len; i++) {
    uint16_t next = (uint16_t)((_head + 1) % RX_BUF_SZ);
    if (next == _tail) {
      // buffer full -> drop oldest
      _tail = (uint16_t)((_tail + 1) % RX_BUF_SZ);
    }
    _rxBuf[_head] = (char)data[i];
    _head = next;
  }
  _unlock();
}

bool MeblockGamepadBLE::_popByte(char& out) {
  _lock();
  if (_tail == _head) { _unlock(); return false; }
  out = _rxBuf[_tail];
  _tail = (uint16_t)((_tail + 1) % RX_BUF_SZ);
  _unlock();
  return true;
}

// Read one full line (ending with '\n'), strip '\r'. Return true if line ready.
bool MeblockGamepadBLE::_readLine(char* out, size_t outSize) {
  char c;
  while (_popByte(c)) {
    if (c == '\n') {
      if (_lineLen && _line[_lineLen - 1] == '\r') _lineLen--;
      _line[_lineLen] = 0;

      strncpy(out, _line, outSize);
      out[outSize - 1] = 0;

      _lineLen = 0;
      return true;
    }

    if (_lineLen < sizeof(_line) - 1) {
      _line[_lineLen++] = c;
    } else {
      // line too long -> reset
      _lineLen = 0;
    }
  }
  return false;
}

int MeblockGamepadBLE::update(uint8_t maxLines) {
  int parsed = 0;
  char line[96];

  while (parsed < (int)maxLines) {
    if (!_readLine(line, sizeof(line))) break;
    if (_handleLine(line)) parsed++;
  }
  return parsed;
}

bool MeblockGamepadBLE::_handleLine(const char* line) {
  if (!line || !line[0]) return false;

  // ping
  if (line[0] == 'P' && line[1] == 0) {
    _sendNotify("PONG\n");
    return false;
  }

  // snapshot
  if (line[0] == 'G' && line[1] == ',') {
    return _parseSnapshot(line);
  }
  return false;
}

// Parse "G," + 14 ints separated by commas:
// G,bX,bO,bS,bT,dU,dD,dL,dR,L1,R1,jLx,jLy,jRx,jRy
bool MeblockGamepadBLE::_parseSnapshot(const char* line) {
  int v[14];
  int idx = 0;
  int num = 0;
  int sign = 1;
  bool inNum = false;

  for (const char* p = line + 2; *p && idx < 14; p++) {
    char c = *p;
    if (c == '-') {
      sign = -1;
    } else if (c >= '0' && c <= '9') {
      num = num * 10 + (c - '0');
      inNum = true;
    } else if (c == ',') {
      if (inNum) v[idx++] = sign * num;
      sign = 1;
      num = 0;
      inNum = false;
    }
  }
  if (idx < 14 && inNum) v[idx++] = sign * num;

  if (idx != 14) return false;

  // buttons: frame order X O S T
  _btn[0] = v[0] ? 1 : 0; // X
  _btn[1] = v[1] ? 1 : 0; // O
  _btn[2] = v[2] ? 1 : 0; // S
  _btn[3] = v[3] ? 1 : 0; // T

  // dpad: U D L R
  _dpad[0] = v[4] ? 1 : 0; // U
  _dpad[1] = v[5] ? 1 : 0; // D
  _dpad[2] = v[6] ? 1 : 0; // L
  _dpad[3] = v[7] ? 1 : 0; // R

  // shoulders: L1 R1
  _sh[0] = v[8] ? 1 : 0;   // L1
  _sh[1] = v[9] ? 1 : 0;   // R1

  // joysticks
  _jlx = _dz(v[10]);
  _jly = _dz(v[11]);
  _jrx = _dz(v[12]);
  _jry = _dz(v[13]);

  _lastSnapshotMs = millis();
  return true;
}

// ===== reads =====
int MeblockGamepadBLE::readButton(char key) const {
  switch (toupper((unsigned char)key)) {
    case 'X': return _btn[0];
    case 'O': return _btn[1];
    case 'S': return _btn[2];
    case 'T': return _btn[3];
    default:  return 0;
  }
}

int MeblockGamepadBLE::readDpad(char dir) const {
  switch (toupper((unsigned char)dir)) {
    case 'U': return _dpad[0];
    case 'D': return _dpad[1];
    case 'L': return _dpad[2];
    case 'R': return _dpad[3];
    default:  return 0;
  }
}

int MeblockGamepadBLE::readShoulder(const char* sh) const {
  if (!sh) return 0;
  if ((sh[0] == 'L' || sh[0] == 'l') && sh[1] == '1' && sh[2] == 0) return _sh[0];
  if ((sh[0] == 'R' || sh[0] == 'r') && sh[1] == '1' && sh[2] == 0) return _sh[1];
  return 0;
}

int MeblockGamepadBLE::joyX(char side) const {
  return (toupper((unsigned char)side) == 'L') ? _jlx : _jrx;
}

int MeblockGamepadBLE::joyY(char side) const {
  return (toupper((unsigned char)side) == 'L') ? _jly : _jry;
}

int MeblockGamepadBLE::joyDistance(char side) const {
  int x = joyX(side), y = joyY(side);
  float d = sqrtf((float)x * x + (float)y * y);
  int di = (int)(d + 0.5f);
  return (di > 100) ? 100 : di;
}

int MeblockGamepadBLE::joyAngle(char side) const {
  int x = joyX(side), y = joyY(side);
  if (x == 0 && y == 0) return 0;

  float a = atan2f((float)y, (float)x) * 180.0f / (float)M_PI;
  if (a < 0) a += 360.0f;
  int ai = (int)a;
  return ai % 360;
}
