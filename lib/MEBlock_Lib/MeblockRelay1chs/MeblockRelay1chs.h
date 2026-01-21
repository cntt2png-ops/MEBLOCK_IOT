#pragma once
#include <Arduino.h>

class MeblockRelay {
public:
  // pin: GPIO điều khiển
  // activeHigh: true = tích cực dương, false = tích cực âm
  MeblockRelay(uint8_t pin, bool activeHigh = true);

  // khởi tạo chân, mặc định relay OFF
  void begin();

  // đóng relay
  void on();

  // ngắt relay
  void off();

private:
  uint8_t _pin;
  bool _activeHigh;

  inline uint8_t levelOn() const {
    return _activeHigh ? HIGH : LOW;
  }

  inline uint8_t levelOff() const {
    return _activeHigh ? LOW : HIGH;
  }
};
