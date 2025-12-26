#include "MeblockServo.h"

MeBlockServo servo;

float MeBlockServo::clampf(float v, float vmin, float vmax) {
  if (v < vmin) return vmin;
  if (v > vmax) return vmax;
  return v;
}

#if defined(ARDUINO_ARCH_ESP32)
MeBlockServo::Slot MeBlockServo::slots[kMaxServos];

MeBlockServo::Slot* MeBlockServo::get_slot(uint8_t pin) {
  for (int i = 0; i < kMaxServos; i++) {
    if (slots[i].used && slots[i].pin == pin) return &slots[i];
  }
  return nullptr;
}

MeBlockServo::Slot* MeBlockServo::alloc_slot(uint8_t pin) {
  for (int i = 0; i < kMaxServos; i++) {
    if (!slots[i].used) {
      slots[i].used = true;
      slots[i].pin = pin;
      slots[i].attached = false;
      return &slots[i];
    }
  }
  return nullptr; // hết slot
}

void MeBlockServo::ensure_attached(Slot* sl, uint16_t freq, uint16_t min_us, uint16_t max_us) {
  if (!sl) return;

  // nếu đang attach mà cấu hình đổi -> detach rồi attach lại
  bool needReattach = (!sl->attached) ||
                      (sl->freq != freq) ||
                      (sl->min_us != min_us) ||
                      (sl->max_us != max_us);

  if (!needReattach) return;

  if (sl->attached) sl->s.detach();

  sl->freq = freq;
  sl->min_us = min_us;
  sl->max_us = max_us;

  sl->s.setPeriodHertz(freq);         // API thường dùng với ESP32Servo :contentReference[oaicite:3]{index=3}
  sl->s.attach(sl->pin, min_us, max_us);
  sl->attached = true;
}
#endif

void MeBlockServo::servo_180(uint8_t pin, float angle, uint16_t freq, uint16_t min_us, uint16_t max_us) {
#if defined(ARDUINO_ARCH_ESP32)
  Slot* sl = get_slot(pin);
  if (!sl) sl = alloc_slot(pin);
  ensure_attached(sl, freq, min_us, max_us);

  float a = clampf(angle, 0.0f, 180.0f);
  float pulse = min_us + (max_us - min_us) * (a / 180.0f);
  sl->s.writeMicroseconds((int)pulse);
#else
  (void)pin; (void)angle; (void)freq; (void)min_us; (void)max_us;
#endif
}

void MeBlockServo::servo_270(uint8_t pin, float angle, uint16_t freq, uint16_t min_us, uint16_t max_us) {
#if defined(ARDUINO_ARCH_ESP32)
  Slot* sl = get_slot(pin);
  if (!sl) sl = alloc_slot(pin);
  ensure_attached(sl, freq, min_us, max_us);

  float a = clampf(angle, 0.0f, 270.0f);
  float pulse = min_us + (max_us - min_us) * (a / 270.0f);
  sl->s.writeMicroseconds((int)pulse);
#else
  (void)pin; (void)angle; (void)freq; (void)min_us; (void)max_us;
#endif
}

void MeBlockServo::servo_360(uint8_t pin, float speed_percent, uint16_t freq,
                             uint16_t min_us, uint16_t max_us, int16_t center_offset_us) {
#if defined(ARDUINO_ARCH_ESP32)
  Slot* sl = get_slot(pin);
  if (!sl) sl = alloc_slot(pin);
  ensure_attached(sl, freq, min_us, max_us);

  float sp = clampf(speed_percent, -100.0f, 100.0f);

  float center = (min_us + max_us) * 0.5f + center_offset_us;
  float span = (max_us - min_us) * 0.5f;
  float pulse = center + span * (sp / 100.0f);

  sl->s.writeMicroseconds((int)pulse);   // writeMicroseconds là cách chuẩn để điều khiển xung :contentReference[oaicite:4]{index=4}
#else
  (void)pin; (void)speed_percent; (void)freq; (void)min_us; (void)max_us; (void)center_offset_us;
#endif
}

void MeBlockServo::servo_off(uint8_t pin) {
#if defined(ARDUINO_ARCH_ESP32)
  Slot* sl = get_slot(pin);
  if (!sl) return;
  if (sl->attached) {
    sl->s.detach();
    sl->attached = false;
  }
#else
  (void)pin;
#endif
}
