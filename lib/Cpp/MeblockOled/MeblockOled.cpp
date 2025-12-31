#include "MeblockOled.h"

static inline uint16_t _col(uint8_t c) { return c ? 1 : 0; }

bool Oled::strEqNoCase(const char *a, const char *b) {
  if (!a || !b) return false;
  while (*a && *b) {
    char ca = *a++, cb = *b++;
    if (ca >= 'a' && ca <= 'z') ca = ca - 'a' + 'A';
    if (cb >= 'a' && cb <= 'z') cb = cb - 'a' + 'A';
    if (ca != cb) return false;
  }
  return (*a == '\0' && *b == '\0');
}

Oled::Oled(uint16_t width, uint16_t height, int sda, int scl, const char *driver, int8_t rstPin)
: _w(width), _h(height),
  _sda(sda), _scl(scl),
  _driver(driver ? driver : "SSD1306"),
  _rstPin(rstPin) {}

Oled::~Oled() {
  if (_ssd) { delete _ssd; _ssd = nullptr; }
  if (_sh)  { delete _sh;  _sh  = nullptr; }
  _gfx = nullptr;
}

bool Oled::scanAutoAddr(uint8_t &outAddr) {
  bool foundAny = false;
  uint8_t firstFound = 0;
  bool has3C = false, has3D = false;

  for (uint8_t a = 0x08; a <= 0x77; a++) {
    Wire.beginTransmission(a);
    uint8_t err = Wire.endTransmission();
    if (err == 0) {
      if (!foundAny) { foundAny = true; firstFound = a; }
      if (a == 0x3C) has3C = true;
      if (a == 0x3D) has3D = true;
    }
  }

  if (!foundAny) return false;
  outAddr = has3C ? 0x3C : (has3D ? 0x3D : firstFound);
  return true;
}

bool Oled::initDriverWithAddr(uint8_t addr) {
  _addrFound = addr;

  // tạo instance theo driver (một lần)
  if (!_gfx) {
    if (strEqNoCase(_driver, "SH1106")) {
      _sh = new Adafruit_SH1106G(_w, _h, &Wire, _rstPin);
      _gfx = _sh;
    } else {
      // default SSD1306
      _ssd = new Adafruit_SSD1306(_w, _h, &Wire, _rstPin);
      _gfx = _ssd;
    }
  }

  bool ok = false;

  if (_sh) {
    ok = _sh->begin(_addrFound, true);
    if (ok) { _sh->clearDisplay(); _sh->display(); }
  } else if (_ssd) {
    // periphBegin=false vì ta đã Wire.begin(sda,scl)
    ok = _ssd->begin(SSD1306_SWITCHCAPVCC, _addrFound, true, false);
    if (ok) { _ssd->clearDisplay(); _ssd->display(); }
  }

  if (ok) {
    _gfx->setTextSize(1);
    _gfx->setTextWrap(false);
  }
  return ok;
}

bool Oled::begin(uint32_t freq) {
  if (_ready) return true;

#if defined(ARDUINO_ARCH_ESP32)
  if (!Wire.begin(_sda, _scl, freq)) {
    _ready = false;
    return false;
  }
#else
  Wire.begin();
  Wire.setClock(freq);
#endif

  uint8_t addr = 0;
  if (!scanAutoAddr(addr)) {
    _addrFound = 0;
    _ready = false;
    return false;
  }

  _ready = initDriverWithAddr(addr);
  return _ready;
}

// ===== API giống micropython =====
void Oled::clear() { fill(0); show(); }

void Oled::fill(uint8_t c) {
  if (_gfx) _gfx->fillScreen(_col(c));
}

void Oled::show() {
  if (_sh) _sh->display();
  else if (_ssd) _ssd->display();
}

void Oled::text(const String &s, int16_t x, int16_t y, uint8_t c) {
  if (!_gfx) return;
  _gfx->setTextSize(1);
  _gfx->setTextColor(_col(c));
  _gfx->setCursor(x, y);
  _gfx->print(s);
}

void Oled::text_wrap(const String &s, int16_t x, int16_t y,
                     uint8_t c, uint8_t line_h, uint8_t char_w) {
  if (!_gfx) return;

  int max_cols = (char_w == 0) ? 1 : max(1, (int)(_w / char_w));
  int16_t cy = y;
  String line;
  line.reserve(max_cols + 4);

  for (size_t i = 0; i < s.length(); i++) {
    char ch = s[i];
    if (ch == '\r') continue;

    bool newLine = (ch == '\n');
    bool overflow = ((int)line.length() >= max_cols);

    if (newLine || overflow) {
      text(line, x, cy, c);
      cy += line_h;
      line = "";
      if (cy > (int16_t)(_h - line_h)) break;
      if (!newLine) line += ch;
    } else {
      line += ch;
    }
  }
  if (line.length() && cy <= (int16_t)(_h - line_h)) text(line, x, cy, c);
}

void Oled::poweroff() {
  if (_sh) _sh->oled_command(0xAE);
  else if (_ssd) _ssd->ssd1306_command(0xAE);
}

void Oled::poweron() {
  if (_sh) _sh->oled_command(0xAF);
  else if (_ssd) _ssd->ssd1306_command(0xAF);
}

void Oled::invert(bool inv) {
  if (_sh) _sh->invertDisplay(inv);
  else if (_ssd) _ssd->invertDisplay(inv);
}

void Oled::contrast(uint8_t val) {
  if (_sh) {
    _sh->setContrast(val);
  } else if (_ssd) {
    _ssd->ssd1306_command(0x81);
    _ssd->ssd1306_command(val);
  }
}

// drawing
void Oled::pixel(int16_t x, int16_t y, uint8_t c) {
  if (_gfx) _gfx->drawPixel(x, y, _col(c));
}
void Oled::line(int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint8_t c) {
  if (_gfx) _gfx->drawLine(x1, y1, x2, y2, _col(c));
}
void Oled::hline(int16_t x, int16_t y, int16_t w, uint8_t c) {
  if (_gfx) _gfx->drawFastHLine(x, y, w, _col(c));
}
void Oled::vline(int16_t x, int16_t y, int16_t h, uint8_t c) {
  if (_gfx) _gfx->drawFastVLine(x, y, h, _col(c));
}
void Oled::rect(int16_t x, int16_t y, int16_t w, int16_t h, uint8_t c) {
  if (_gfx) _gfx->drawRect(x, y, w, h, _col(c));
}
void Oled::fill_rect(int16_t x, int16_t y, int16_t w, int16_t h, uint8_t c) {
  if (_gfx) _gfx->fillRect(x, y, w, h, _col(c));
}
