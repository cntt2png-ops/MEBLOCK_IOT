#include "MeblockDht.h"

// ===== Dht20Sensor (AHT20 over I2C) =====

Dht20Sensor::Dht20Sensor(int8_t sda, int8_t scl,
                         uint8_t addr, TwoWire &wire)
  : _sda(sda),
    _scl(scl),
    _addr(addr),
    _wire(&wire),
    _lastOk(false),
    _t(NAN),
    _h(NAN)
{}

bool Dht20Sensor::begin(uint32_t freq) {
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

  // Adafruit_AHTX0: mặc định address 0x38, cho phép chọn bus khác qua tham số Wire*
  if (!_aht.begin(_wire)) {
    _lastOk = false;
    return false;
  }

  _lastOk = true;
  return true;
}

bool Dht20Sensor::read() {
  sensors_event_t humEvent, tempEvent;
  _aht.getEvent(&humEvent, &tempEvent);

  if (isnan(humEvent.relative_humidity) || isnan(tempEvent.temperature)) {
    _lastOk = false;
    return false;
  }

  _h = humEvent.relative_humidity;
  _t = tempEvent.temperature;
  _lastOk = true;
  return true;
}

// ===== Dht11Sensor (Adafruit DHT) =====

Dht11Sensor::Dht11Sensor(uint8_t pin)
  : _pin(pin),
    _dht(pin, DHT11),
    _lastOk(false),
    _t(NAN),
    _h(NAN)
{}

bool Dht11Sensor::begin() {
  _dht.begin();
  _lastOk = false;
  return true;
}

bool Dht11Sensor::read() {
  float h = _dht.readHumidity();
  float t = _dht.readTemperature();  // °C

  if (isnan(h) || isnan(t)) {
    _lastOk = false;
    return false;
  }

  _h = h;
  _t = t;
  _lastOk = true;
  return true;
}
