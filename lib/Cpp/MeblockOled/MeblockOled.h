#pragma once
#include <Arduino.h>
#include <Wire.h>

#define SH110X_NO_SPLASH
#ifndef SSD1306_NO_SPLASH
#define SSD1306_NO_SPLASH
#endif

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_SH110X.h>

// preset giống micropython S3 DevKitC-1 bus0
#ifndef S3_I2C0_SDA
#define S3_I2C0_SDA 8
#define S3_I2C0_SCL 9
#endif

class Oled {
public:
  // CHỈ CÒN: kích thước + SDA/SCL + driver name
  // driver: "SSD1306" (default) hoặc "SH1106"
  Oled(uint16_t width = 128, uint16_t height = 64,
       int sda = S3_I2C0_SDA, int scl = S3_I2C0_SCL,
       const char *driver = "SSD1306",
       int8_t rstPin = -1);

  ~Oled();

  // tự begin Wire + scan addr + init driver
  bool begin(uint32_t freq = 400000UL);

  // API giống micropython wrapper
  void clear();                 // fill(0) + show()
  void fill(uint8_t c = 1);     // 0/1
  void show();
  void text(const String &s, int16_t x, int16_t y, uint8_t c = 1);
  void text_wrap(const String &s, int16_t x = 0, int16_t y = 0,
                 uint8_t c = 1, uint8_t line_h = 8, uint8_t char_w = 8);

  void poweroff();
  void poweron();
  void invert(bool inv);
  void contrast(uint8_t val);

  // drawing helpers
  void pixel(int16_t x, int16_t y, uint8_t c = 1);
  void line(int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint8_t c = 1);
  void hline(int16_t x, int16_t y, int16_t w, uint8_t c = 1);
  void vline(int16_t x, int16_t y, int16_t h, uint8_t c = 1);
  void rect(int16_t x, int16_t y, int16_t w, int16_t h, uint8_t c = 1);
  void fill_rect(int16_t x, int16_t y, int16_t w, int16_t h, uint8_t c = 1);

  // info
  uint8_t address() const { return _addrFound; }
  bool ready() const { return _ready; }
  const char *driver() const { return _driver; }

  // raw access (nếu cần)
  Adafruit_GFX &dev() { return *_gfx; }
  Adafruit_SSD1306 *ssd1306() { return _ssd; }
  Adafruit_SH1106G *sh1106() { return _sh; }

private:
  uint16_t _w, _h;
  int _sda, _scl;
  const char *_driver;
  int8_t _rstPin;

  uint8_t _addrFound = 0;
  bool _ready = false;

  Adafruit_GFX *_gfx = nullptr;
  Adafruit_SSD1306 *_ssd = nullptr;
  Adafruit_SH1106G *_sh = nullptr;

private:
  bool scanAutoAddr(uint8_t &outAddr);
  bool initDriverWithAddr(uint8_t addr);
  static bool strEqNoCase(const char *a, const char *b);
};
