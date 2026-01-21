#include "MeblockMQ3.h"

// ===== helpers =====
static inline int clampi(int v, int lo, int hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

BaseMQ::BaseMQ(int pinData, float boardResistance, float baseVoltage, int measuringStrategy, bool invert)
: _pinData(pinData),
  _boardResistance(boardResistance),
  _baseVoltage(baseVoltage),
  _measuringStrategy(measuringStrategy),
  _invert(invert) {}

void BaseMQ::begin() {
  if (_inited) return;
  _inited = true;

#if defined(ARDUINO_ARCH_ESP32)
  // ESP32 ADC: set 12-bit, atten để đọc gần 3.3V
  analogReadResolution(12);
  // Prefer per-pin attenuation if available
  #if defined(ADC_11db)
    analogSetPinAttenuation(_pinData, ADC_11db);
  #endif
#endif

  pinMode(_pinData, INPUT);
}

void BaseMQ::setSlowMode(bool slow) {
  _measuringStrategy = slow ? STRATEGY_SLOW : STRATEGY_FAST;
}

void BaseMQ::setInvert(bool invert) {
  _invert = invert;
}

void BaseMQ::setCalibration(float ro) {
  _ro = ro;
}

int BaseMQ::readAdcOnce() {
  // Arduino ESP32 analogRead trả 0..4095 (khi resolution=12)
  int v = analogRead(_pinData);
  v = clampi(v, 0, 4095);
  if (_invert) v = 4095 - v;
  return v;
}

float BaseMQ::readAdcAvg() {
  const int k = (_measuringStrategy == STRATEGY_SLOW) ? _samplesSlow : _samplesFast;
  uint32_t s = 0;
  for (int i = 0; i < k; i++) s += (uint32_t)readAdcOnce();
  return (k > 0) ? ((float)s / (float)k) : (float)readAdcOnce();
}

float BaseMQ::calcRs(float adcValue) {
  if (adcValue <= 0.0f) adcValue = 1.0f;

  float vout = (adcValue / 4095.0f) * _baseVoltage;
  if (vout <= 0.0f) vout = 0.0001f;

  float rs = _boardResistance * (_baseVoltage - vout) / vout;
  if (rs < 0.0001f) rs = 0.0001f;
  return rs;
}

float BaseMQ::rsRoRatio() {
  // Chưa calib -> trả -1 để dễ phát hiện
  if (_ro <= 0.0f) return -1.0f;

  const float adc = readAdcAvg();
  const float rs  = calcRs(adc);

  float r = rs / _ro;
  if (r < 0.000001f) r = 0.000001f;
  return r;
}

float BaseMQ::readScaled(float m, float b) {
  const float r = rsRoRatio();
  if (r <= 0.0f) return -1.0f; // not calibrated

  // x = m*log10(r) + b ; return 10^x
  const float x = (m * log10f(r)) + b;
  return powf(10.0f, x);
}

float BaseMQ::calibrate(uint32_t durationMs) {
  begin();
  durationMs = (uint32_t)clampi((int)durationMs, 200, 5000);

  const uint32_t endMs = millis() + durationMs;

  uint32_t s = 0;
  uint32_t n = 0;

  while ((int32_t)(endMs - millis()) > 0) {
    s += (uint32_t)readAdcOnce();
    n++;
    delay(10);
  }

  const float adc = (n > 0) ? ((float)s / (float)n) : (float)readAdcOnce();
  const float rs  = calcRs(adc);

  // ro_air = rs / RoInCleanAir
  const float roAir = rs / getRoInCleanAir();
  _ro = roAir;
  return _ro;
}

// ===== MQ3 =====
MQ3::MQ3(int pinData, float boardResistance, float baseVoltage, bool invert)
: BaseMQ(pinData, boardResistance, baseVoltage, STRATEGY_FAST, invert) {}

float MQ3::readAlcoholMgL() {
  // Port y hệt mq3.py:
  // mgL = readScaled(-0.66, -0.62) * 50.0 ; clamp 0.005..10.0
  const float v = readScaled(-0.66f, -0.62f);
  if (v <= 0.0f) return -1.0f; // not calibrated

  float mgL = v * 50.0f;
  if (mgL < 0.005f) mgL = 0.005f;
  else if (mgL > 10.0f) mgL = 10.0f;
  return mgL;
}

float MQ3::readAlcoholPpm() {
  const float mgL = readAlcoholMgL();
  if (mgL <= 0.0f) return -1.0f;
  return mgL * 1000.0f;
}
