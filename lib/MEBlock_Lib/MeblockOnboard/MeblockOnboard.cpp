#include "MeblockOnboard.h"

// ---------- Board detection ----------
static bool _isESP32S3() {
#if defined(ARDUINO_ARCH_ESP32)
  #if defined(CONFIG_IDF_TARGET_ESP32S3) || defined(ARDUINO_ESP32S3_DEV) || defined(ARDUINO_ESP32S3_BOX) || defined(ARDUINO_ESP32S3)
    return true;
  #endif
#endif
  return false;
}

static bool _isESP32C3() {
#if defined(ARDUINO_ARCH_ESP32)
  #if defined(CONFIG_IDF_TARGET_ESP32C3) || defined(ARDUINO_ESP32C3_DEV) || defined(ARDUINO_ESP32C3)
    return true;
  #endif
#endif
  return false;
}

MeblockOnboardProfile meblockDetectOnboardProfile() {
  MeblockOnboardProfile p;
  p.rgbCount = 1;
  p.order = MB_ORDER_GRB;

  if (_isESP32S3())      p.rgbPin = 48;
  else if (_isESP32C3()) p.rgbPin = 8;
  else                   p.rgbPin = 2;

  return p;
}

// ---------- MeblockOnboardRGB ----------
MeblockOnboardRGB::MeblockOnboardRGB() {}

void MeblockOnboardRGB::_applyOrder(uint32_t &neoType) const {
  neoType = NEO_KHZ800;
  switch (_order) {
    case MB_ORDER_GRB: neoType |= NEO_GRB; break;
    case MB_ORDER_RGB: neoType |= NEO_RGB; break;
    case MB_ORDER_BRG: neoType |= NEO_BRG; break;
    case MB_ORDER_RBG: neoType |= NEO_RBG; break;
    case MB_ORDER_GBR: neoType |= NEO_GBR; break;
    case MB_ORDER_BGR: neoType |= NEO_BGR; break;
    default:           neoType |= NEO_GRB; break;
  }
}

void MeblockOnboardRGB::_ensureReady() {
  if (_ready && _px) return;

  uint32_t neoType = 0;
  _applyOrder(neoType);

  if (_px) { delete _px; _px = nullptr; }
  _px = new Adafruit_NeoPixel(_count, _pin, neoType);

  _px->begin();
  _px->setBrightness(_brightness);
  _px->clear();
  _px->show();

  _ready = true;
}

bool MeblockOnboardRGB::begin() {
  MeblockOnboardProfile p = meblockDetectOnboardProfile();
  return begin(p.rgbPin, p.rgbCount, p.order);
}

bool MeblockOnboardRGB::begin(int pin, uint16_t count, MeblockNeoOrder order) {
  _pin = pin;
  _count = (count == 0) ? 1 : count;
  _order = order;

  _ensureReady();
  return _ready;
}

void MeblockOnboardRGB::brightness(uint8_t b) {
  _brightness = b;
  if (_px) _px->setBrightness(_brightness);
}

void MeblockOnboardRGB::_setAll(uint8_t r, uint8_t g, uint8_t b) {
  if (!_ready) _ensureReady();
  if (!_px) return;

  for (uint16_t i = 0; i < _count; i++) {
    _px->setPixelColor(i, _px->Color(r, g, b));
  }
}

void MeblockOnboardRGB::show() {
  if (!_ready) _ensureReady();
  if (_px) _px->show();
}

void MeblockOnboardRGB::rgb(uint8_t r, uint8_t g, uint8_t b) {
  _lastR = r; _lastG = g; _lastB = b;
  _setAll(r, g, b);
  show();
}

static void _presetToRGB(MeblockColor c, uint8_t &r, uint8_t &g, uint8_t &b) {
  switch (c) {
    case MB_COLOR_OFF:    r=0;   g=0;   b=0;   break;
    case MB_COLOR_WHITE:  r=255; g=255; b=255; break;
    case MB_COLOR_RED:    r=255; g=0;   b=0;   break;
    case MB_COLOR_GREEN:  r=0;   g=255; b=0;   break;
    case MB_COLOR_BLUE:   r=0;   g=0;   b=255; break;
    case MB_COLOR_YELLOW: r=255; g=255; b=0;   break;
    case MB_COLOR_PURPLE: r=128; g=0;   b=255; break;
    case MB_COLOR_CYAN:   r=0;   g=255; b=255; break;
    case MB_COLOR_ORANGE: r=255; g=64;  b=0;   break;
    case MB_COLOR_PINK:   r=255; g=0;   b=64;  break;
    default:              r=255; g=255; b=255; break;
  }
}

void MeblockOnboardRGB::color(MeblockColor c) {
  uint8_t r,g,b;
  _presetToRGB(c, r,g,b);
  if (c == MB_COLOR_OFF) { off(); return; }
  rgb(r,g,b);
}

void MeblockOnboardRGB::on() {
  rgb(_lastR, _lastG, _lastB);
}

void MeblockOnboardRGB::off() {
  _setAll(0,0,0);
  show();
}

void MeblockOnboardRGB::toggle() {
  if (_lastR == 0 && _lastG == 0 && _lastB == 0) {
    _lastR = 255; _lastG = 255; _lastB = 255;
    on();
  } else {
    uint8_t r = _lastR, g = _lastG, b = _lastB;
    off();
    _lastR = r; _lastG = g; _lastB = b;
  }
}

// --------- HEX helpers ----------
uint8_t MeblockOnboardRGB::_hexNibble(char c){
  if (c>='0' && c<='9') return (uint8_t)(c-'0');
  if (c>='a' && c<='f') return (uint8_t)(10 + (c-'a'));
  if (c>='A' && c<='F') return (uint8_t)(10 + (c-'A'));
  return 0;
}
uint8_t MeblockOnboardRGB::_hexByte(const String& s, int i){
  if (s.length() < i+2) return 0;
  return (uint8_t)((_hexNibble(s[i]) << 4) | _hexNibble(s[i+1]));
}
bool MeblockOnboardRGB::_parseHex(const String& s, uint8_t &r, uint8_t &g, uint8_t &b){
  String c = s;
  c.trim();
  if (c.length() < 7) return false;
  if (c[0] == '#') {
    r = _hexByte(c, 1);
    g = _hexByte(c, 3);
    b = _hexByte(c, 5);
    return true;
  }
  // also accept "0xRRGGBB"
  if (c.length() >= 8 && (c.startsWith("0x") || c.startsWith("0X"))) {
    r = _hexByte(c, 2);
    g = _hexByte(c, 4);
    b = _hexByte(c, 6);
    return true;
  }
  return false;
}

String MeblockOnboardRGB::makeColor(int r, int g, int b){
  r = constrain(r, 0, 255);
  g = constrain(g, 0, 255);
  b = constrain(b, 0, 255);
  char buf[8];
  snprintf(buf, sizeof(buf), "#%02X%02X%02X", r, g, b);
  return String(buf);
}

void MeblockOnboardRGB::hex(const String& hexColor){
  uint8_t r=0,g=0,b=0;
  if (_parseHex(hexColor, r,g,b)) rgb(r,g,b);
}

void MeblockOnboardRGB::blink(uint16_t times, uint16_t periodMs, MeblockColor c) {
  if (times < 1) return;
  if (periodMs < 10) periodMs = 10;

  uint16_t onMs = periodMs / 2;
  uint16_t offMs = periodMs - onMs;

  uint8_t sr = _lastR, sg = _lastG, sb = _lastB;

  for (uint16_t i=0; i<times; i++) {
    color(c);
    delay(onMs);
    off();
    delay(offMs);
  }

  _lastR = sr; _lastG = sg; _lastB = sb;
}

void MeblockOnboardRGB::blinkRGB(uint8_t r, uint8_t g, uint8_t b, uint16_t times, uint16_t periodMs, bool endOff) {
  if (times < 1) return;
  if (periodMs < 10) periodMs = 10;

  uint16_t onMs = periodMs / 2;
  uint16_t offMs = periodMs - onMs;

  uint8_t sr = _lastR, sg = _lastG, sb = _lastB;

  for (uint16_t i=0; i<times; i++) {
    rgb(r,g,b);
    delay(onMs);
    off();
    delay(offMs);
  }

  if (!endOff) rgb(r,g,b);

  _lastR = sr; _lastG = sg; _lastB = sb;
}

void MeblockOnboardRGB::blinkHex(const String& hexColor, uint16_t times, uint16_t periodMs, bool endOff) {
  uint8_t r=0,g=0,b=0;
  if (!_parseHex(hexColor, r,g,b)) return;
  blinkRGB(r,g,b, times, periodMs, endOff);
}

// ---------- Global singleton ----------
MeblockOnboardRGB OnboardRGB;
