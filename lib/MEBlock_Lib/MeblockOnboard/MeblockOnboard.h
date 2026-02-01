#pragma once
/*
  MeblockOnboard - Onboard RGB LED helper (Arduino C++) v0.5.1
  -----------------------------------------------------------
  IMPORTANT: This library ALWAYS includes Adafruit_NeoPixel.

  - Auto-detect board (ESP32-S3 / ESP32-C3 / ESP32) and choose default RGB pin
    * ESP32-S3: GPIO48 (example as requested)
    * ESP32-C3: GPIO8  (common dev boards; override if your board differs)
    * ESP32   : GPIO2  (common dev boards; override if your board differs)

  - API:
    + on/off/toggle
    + preset colors: color(MB_COLOR_*)
    + raw rgb(r,g,b)
    + makeColor(r,g,b) -> "#RRGGBB"
    + hex("#RRGGBB")   -> show LED by created color
    + blink preset / blinkRGB / blinkHex (blocking)

  Dependency:
    - Adafruit NeoPixel library (required)
*/

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

// ---------- Preset Colors ----------
enum MeblockColor : uint8_t {
  MB_COLOR_OFF = 0,
  MB_COLOR_WHITE,
  MB_COLOR_RED,
  MB_COLOR_GREEN,
  MB_COLOR_BLUE,
  MB_COLOR_YELLOW,
  MB_COLOR_PURPLE,
  MB_COLOR_CYAN,
  MB_COLOR_ORANGE,
  MB_COLOR_PINK,
};

// ---------- Color Order ----------
enum MeblockNeoOrder : uint8_t {
  MB_ORDER_GRB = 0,
  MB_ORDER_RGB,
  MB_ORDER_BRG,
  MB_ORDER_RBG,
  MB_ORDER_GBR,
  MB_ORDER_BGR,
};

// ---------- Board profile ----------
struct MeblockOnboardProfile {
  int      rgbPin;
  uint16_t rgbCount;
  MeblockNeoOrder order;
};

MeblockOnboardProfile meblockDetectOnboardProfile();

// ---------- Onboard RGB ----------
class MeblockOnboardRGB {
public:
  MeblockOnboardRGB();

  bool begin();  // auto
  bool begin(int pin, uint16_t count = 1, MeblockNeoOrder order = MB_ORDER_GRB);

  // Basic control
  void on();
  void off();
  void toggle();

  // Set color
  void rgb(uint8_t r, uint8_t g, uint8_t b);
  void color(MeblockColor c);
  void brightness(uint8_t b); // 0..255
  void show();

  // Create/show by hex color
  static String makeColor(int r, int g, int b);     // "#RRGGBB"
  void hex(const String& hexColor);                 // show LED by "#RRGGBB"

  // Effects (blocking)
  void blink(uint16_t times = 3, uint16_t periodMs = 200, MeblockColor c = MB_COLOR_WHITE);
  void blinkRGB(uint8_t r, uint8_t g, uint8_t b, uint16_t times = 3, uint16_t periodMs = 200, bool endOff = true);
  void blinkHex(const String& hexColor, uint16_t times = 3, uint16_t periodMs = 200, bool endOff = true);

  // Info
  bool available() const { return _ready; }
  int  pin() const { return _pin; }
  uint16_t count() const { return _count; }
  MeblockNeoOrder order() const { return _order; }
  uint8_t lastR() const { return _lastR; }
  uint8_t lastG() const { return _lastG; }
  uint8_t lastB() const { return _lastB; }

private:
  void _applyOrder(uint32_t &neoType) const;
  void _setAll(uint8_t r, uint8_t g, uint8_t b);
  void _ensureReady();

  static uint8_t _hexNibble(char c);
  static uint8_t _hexByte(const String& s, int i);
  static bool _parseHex(const String& s, uint8_t &r, uint8_t &g, uint8_t &b);

private:
  bool _ready = false;

  int _pin = -1;
  uint16_t _count = 1;
  MeblockNeoOrder _order = MB_ORDER_GRB;

  uint8_t _brightness = 64;

  uint8_t _lastR = 255;
  uint8_t _lastG = 255;
  uint8_t _lastB = 255;

  Adafruit_NeoPixel* _px = nullptr;
};

extern MeblockOnboardRGB OnboardRGB;

namespace meblock_onboard {
  inline bool begin() { return OnboardRGB.begin(); }
  inline void on() { OnboardRGB.on(); }
  inline void off() { OnboardRGB.off(); }
  inline void color(MeblockColor c) { OnboardRGB.color(c); }
  inline void rgb(uint8_t r,uint8_t g,uint8_t b) { OnboardRGB.rgb(r,g,b); }
  inline String makeColor(int r,int g,int b) { return MeblockOnboardRGB::makeColor(r,g,b); }
  inline void hex(const String& c) { OnboardRGB.hex(c); }
  inline void blinkHex(const String& c, uint16_t times=3, uint16_t periodMs=200, bool endOff=true) { OnboardRGB.blinkHex(c,times,periodMs,endOff); }
}
