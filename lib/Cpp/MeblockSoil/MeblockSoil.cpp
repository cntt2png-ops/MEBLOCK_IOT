#include "MeblockSoil.h"

#if defined(ARDUINO_ARCH_ESP32)
  // Arduino-ESP32 có analogSetPinAttenuation / ADC_11db (tuỳ core)
  #ifndef ADC_11db
    // nếu core không define, bỏ qua (vẫn chạy)
  #endif
#endif

// ===== utils =====
int SoilMoisture::clampi(int v, int lo, int hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

SoilMoisture::SoilMoisture(int pin, int dry_adc, int wet_adc, int samples, bool invert)
: _pin(pin),
  _dry(clampi(dry_adc, 0, 4095)),
  _wet(clampi(wet_adc, 0, 4095)),
  _samples(clampi(samples, 1, 50)),
  _invert(invert) {
  pinMode(_pin, INPUT);
  // không begin riêng: readPercent() sẽ tự đảm bảo config ADC
}

void SoilMoisture::setCalib(int dry_adc, int wet_adc) {
  _dry = clampi(dry_adc, 0, 4095);
  _wet = clampi(wet_adc, 0, 4095);
}

void SoilMoisture::setSamples(int samples) {
  _samples = clampi(samples, 1, 50);
}

void SoilMoisture::setInvert(bool invert) {
  _invert = invert;
}

void SoilMoisture::ensureAdcConfig() {
#if defined(ARDUINO_ARCH_ESP32)
  // cấu hình 1 lần cho toàn hệ (giống soil.py set width_bits=12, atten_db=11)
  static bool s_inited = false;
  if (!s_inited) {
    analogReadResolution(12);
    s_inited = true;
  }
  // set attenuation theo pin (nếu có API)
  #if defined(ADC_11db)
    // tránh gọi lặp quá nhiều: cache vài pin
    struct PinCache { int pin; bool ok; };
    static PinCache cache[8] = {{-1,false},{-1,false},{-1,false},{-1,false},{-1,false},{-1,false},{-1,false},{-1,false}};
    bool seen = false;
    for (auto &c : cache) { if (c.ok && c.pin == _pin) { seen = true; break; } }
    if (!seen) {
      analogSetPinAttenuation(_pin, ADC_11db);
      for (auto &c : cache) {
        if (!c.ok) { c.pin = _pin; c.ok = true; break; }
      }
    }
  #endif
#else
  // non-ESP32: dùng analogRead mặc định
#endif
}

int SoilMoisture::readAvgRaw() {
  ensureAdcConfig();

  uint32_t s = 0;
  for (int i = 0; i < _samples; i++) {
    int v = analogRead(_pin);
    v = clampi(v, 0, 4095);
    s += (uint32_t)v;
    delay(2); // giống soil.py sleep_ms(2)
  }

  int raw = (int)(s / (uint32_t)_samples);
  raw = clampi(raw, 0, 4095);

  if (_invert) raw = 4095 - raw;
  return raw;
}

int SoilMoisture::readPercent() {
  int raw = readAvgRaw();

  int a = _dry;
  int b = _wet;

  if (a == b) return 0;

  // giống soil.py: nếu a < b thì swap để a là "dry" lớn hơn
  if (a < b) { int t = a; a = b; b = t; }

  if (raw >= a) return 0;
  if (raw <= b) return 100;

  int pct = (int)((long)(a - raw) * 100L / (long)(a - b));
  return clampi(pct, 0, 100);
}
