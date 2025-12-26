#pragma once
#include <Arduino.h>

#if defined(ARDUINO_ARCH_ESP32)
  #include <ESP32Servo.h>
#endif

class MeBlockServo {
public:
  // Tương thích tham số như Python (có default)
  void servo_180(uint8_t pin, float angle,
                 uint16_t freq = 50, uint16_t min_us = 500, uint16_t max_us = 2500);

  void servo_270(uint8_t pin, float angle,
                 uint16_t freq = 50, uint16_t min_us = 500, uint16_t max_us = 2500);

  void servo_360(uint8_t pin, float speed_percent,
                 uint16_t freq = 50, uint16_t min_us = 1000, uint16_t max_us = 2000,
                 int16_t center_offset_us = 0);

  void servo_off(uint8_t pin);

private:
  static float clampf(float v, float vmin, float vmax);

#if defined(ARDUINO_ARCH_ESP32)
  struct Slot {
    bool used = false;
    uint8_t pin = 255;
    uint16_t freq = 50;
    uint16_t min_us = 500;
    uint16_t max_us = 2500;
    Servo s;
    bool attached = false;
  };

  static constexpr int kMaxServos = 16; // thường đủ cho giáo dục
  static Slot slots[kMaxServos];

  static Slot* get_slot(uint8_t pin);
  static Slot* alloc_slot(uint8_t pin);
  static void ensure_attached(Slot* sl, uint16_t freq, uint16_t min_us, uint16_t max_us);
#endif
};

extern MeBlockServo servo;
