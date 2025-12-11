#include "MeblockOled.h"

Oled::Oled(uint16_t width,
           uint16_t height,
           int8_t sda,
           int8_t scl,
           uint8_t addr,
           TwoWire &wire)
  : _width(width),
    _height(height),
    _sda(sda),
    _scl(scl),
    _addr(addr),
    _wire(&wire),
    _display(width, height, &wire, -1) // reset pin = -1
{
}

bool Oled::begin(uint32_t freq) {
  // Khởi tạo I2C
#if defined(ARDUINO_ARCH_ESP32)
  if (_sda >= 0 && _scl >= 0) {
    _wire->begin(_sda, _scl, freq);
  } else {
    _wire->begin();
    _wire->setClock(freq);
  }
#else
  (void)freq;
  _wire->begin();
#endif

  delay(100); // cho ổn định nguồn

  // false = không reset cứng, đúng với nhiều module SH1106
  if (!_display.begin(_addr, false)) {
    // Không tìm thấy màn, return false để user tự xử lý
    return false;
  }

  _display.clearDisplay();
  _display.setTextSize(1);
  _display.setTextColor(SH110X_WHITE);
  _display.setTextWrap(false); // wrap custom do mình xử lý
  _display.display();
  return true;
}

void Oled::clear() {
  fill(false);   // fill đen
  show();
}

void Oled::fill(bool color) {
  _display.fillRect(0, 0, _width, _height,
                    color ? SH110X_WHITE : SH110X_BLACK);
}

void Oled::show() {
  _display.display();
}

void Oled::text(const String &s,
                int16_t x, int16_t y,
                bool color) {
  _display.setCursor(x, y);
  _display.setTextColor(color ? SH110X_WHITE : SH110X_BLACK);
  _display.print(s);
}

// Giống text_wrap bên MicroPython: tự xuống dòng khi quá rộng
void Oled::textWrap(const String &s,
                    int16_t x, int16_t y,
                    bool color,
                    uint8_t lineHeight,
                    uint8_t charWidth) {
  int16_t maxCols = max<int16_t>(1, _width / charWidth);
  int16_t cy = y;
  String line;

  for (size_t i = 0; i < s.length(); ++i) {
    char ch = s[i];
    bool needBreak = (ch == '\n') || (line.length() >= (size_t)maxCols);

    if (needBreak) {
      _display.setCursor(x, cy);
      _display.setTextColor(color ? SH110X_WHITE : SH110X_BLACK);
      _display.print(line);

      cy += lineHeight;
      if (cy > (int16_t)(_height - lineHeight)) {
        return;   // không còn chỗ
      }

      if (ch == '\n') {
        line = "";
      } else {
        line = String(ch);
      }
    } else {
      line += ch;
    }
  }

  if (line.length() && cy <= (int16_t)(_height - lineHeight)) {
    _display.setCursor(x, cy);
    _display.setTextColor(color ? SH110X_WHITE : SH110X_BLACK);
    _display.print(line);
  }
}

void Oled::powerOff() {
  // Thư viện không expose lệnh OFF trực tiếp,
  // ta giả lập tắt bằng cách giảm contrast về 0.
  _display.setContrast(0);
  _display.display();
}

void Oled::powerOn() {
  _display.setContrast(255);
  _display.display();
}

void Oled::invert(bool inv) {
  _display.invertDisplay(inv);
}

void Oled::contrast(uint8_t value) {
  _display.setContrast(value);
}

// ==== helper vẽ primitive giống framebuf ====
void Oled::pixel(int16_t x, int16_t y, bool color) {
  _display.drawPixel(x, y, color ? SH110X_WHITE : SH110X_BLACK);
}

void Oled::line(int16_t x0, int16_t y0,
                int16_t x1, int16_t y1,
                bool color) {
  _display.drawLine(x0, y0, x1, y1,
                    color ? SH110X_WHITE : SH110X_BLACK);
}

void Oled::hline(int16_t x, int16_t y,
                 int16_t w, bool color) {
  _display.drawLine(x, y, x + w - 1, y,
                    color ? SH110X_WHITE : SH110X_BLACK);
}

void Oled::vline(int16_t x, int16_t y,
                 int16_t h, bool color) {
  _display.drawLine(x, y, x, y + h - 1,
                    color ? SH110X_WHITE : SH110X_BLACK);
}

void Oled::rect(int16_t x, int16_t y,
                int16_t w, int16_t h,
                bool color) {
  _display.drawRect(x, y, w, h,
                    color ? SH110X_WHITE : SH110X_BLACK);
}

void Oled::fillRect(int16_t x, int16_t y,
                    int16_t w, int16_t h,
                    bool color) {
  _display.fillRect(x, y, w, h,
                    color ? SH110X_WHITE : SH110X_BLACK);
}
