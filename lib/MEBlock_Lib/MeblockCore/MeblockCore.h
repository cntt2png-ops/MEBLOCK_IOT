#pragma once
/*
  MeblockCore - standalone core-only library (Arduino C++)
  --------------------------------------------------------.
  Chỉ cần 2 file MeblockCore.h/.cpp là có sẵn:
  - Core boot/UART command handlers
  - Onboard RGB LED helper
  - UI frame log helpers: send_inf/send_war/send_err
  - System info helper: meblock_send_info

  KHÔNG cần MeblockOnboard.h/.cpp riêng nữa.
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
  int rgbPin;
  uint16_t rgbCount;
  MeblockNeoOrder order;
};

MeblockOnboardProfile meblockDetectOnboardProfile();

// ---------- Onboard RGB ----------
class MeblockOnboardRGB {
public:
  MeblockOnboardRGB();

  bool begin();
  bool begin(int pin, uint16_t count = 1, MeblockNeoOrder order = MB_ORDER_GRB);

  void on();
  void off();
  void toggle();

  void rgb(uint8_t r, uint8_t g, uint8_t b);
  void color(MeblockColor c);
  void brightness(uint8_t b);
  void show();

  static String makeColor(int r, int g, int b);
  void hex(const String& hexColor);

  void blink(uint16_t times = 3, uint16_t periodMs = 200, MeblockColor c = MB_COLOR_WHITE);
  void blinkRGB(uint8_t r, uint8_t g, uint8_t b, uint16_t times = 3, uint16_t periodMs = 200, bool endOff = true);
  void blinkHex(const String& hexColor, uint16_t times = 3, uint16_t periodMs = 200, bool endOff = true);

  bool available() const { return _ready; }
  int pin() const { return _pin; }
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

// ---------- UI frame log helpers ----------
String meblock_sanitize_frame_token(const String& value);
String meblock_build_frame(const String& type, const String& code, const String& text);
size_t meblock_send_frame(const String& type, const String& code, const String& text, Stream* out = nullptr);

size_t send_inf(const String& codeOrText, const String& text = String(), Stream* out = nullptr);
size_t send_war(const String& codeOrText, const String& text = String(), Stream* out = nullptr);
size_t send_err(const String& codeOrText, const String& text = String(), Stream* out = nullptr);

// ---------- Core ----------
void meblock_core_setup(uint32_t serialBaud = 115200);
void meblock_core_loop();

// ---------- System info ----------
String meblock_build_info_text();
size_t meblock_send_info(Stream* out = nullptr, const String& code = "I_INFO");

namespace meblock_onboard {
  inline bool begin() { return OnboardRGB.begin(); }
  inline void on() { OnboardRGB.on(); }
  inline void off() { OnboardRGB.off(); }
  inline void color(MeblockColor c) { OnboardRGB.color(c); }
  inline void rgb(uint8_t r, uint8_t g, uint8_t b) { OnboardRGB.rgb(r, g, b); }
  inline String makeColor(int r, int g, int b) { return MeblockOnboardRGB::makeColor(r, g, b); }
  inline void hex(const String& c) { OnboardRGB.hex(c); }
  inline void blinkHex(const String& c, uint16_t times = 3, uint16_t periodMs = 200, bool endOff = true) {
    OnboardRGB.blinkHex(c, times, periodMs, endOff);
  }
  inline size_t send_inf(const String& codeOrText, const String& text = String(), Stream* out = nullptr) {
    return ::send_inf(codeOrText, text, out);
  }
  inline size_t send_war(const String& codeOrText, const String& text = String(), Stream* out = nullptr) {
    return ::send_war(codeOrText, text, out);
  }
  inline size_t send_err(const String& codeOrText, const String& text = String(), Stream* out = nullptr) {
    return ::send_err(codeOrText, text, out);
  }
  inline size_t send_info(Stream* out = nullptr, const String& code = "I_INFO") {
    return ::meblock_send_info(out, code);
  }

  // alias ngắn, giữ tương thích nếu đã dùng bản trước
  inline size_t inf(const String& codeOrText, const String& text = String(), Stream* out = nullptr) {
    return ::send_inf(codeOrText, text, out);
  }
  inline size_t war(const String& codeOrText, const String& text = String(), Stream* out = nullptr) {
    return ::send_war(codeOrText, text, out);
  }
  inline size_t err(const String& codeOrText, const String& text = String(), Stream* out = nullptr) {
    return ::send_err(codeOrText, text, out);
  }
}
