#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_AHTX0.h>   // DHT20 / AHT20
#include <DHT.h>              // DHT11

// Preset cho ESP32-S3 DevKitC-1 (giống file MicroPython của bạn)
#ifndef MEBLOCK_S3_I2C0_SDA
#define MEBLOCK_S3_I2C0_SDA 8
#define MEBLOCK_S3_I2C0_SCL 9
#endif

#ifndef MEBLOCK_S3_DHT11_PIN
#define MEBLOCK_S3_DHT11_PIN 4
#endif

// ========== DHT20 / AHT20 I2C ==========
class Dht20Sensor {
public:
  Dht20Sensor(int8_t sda = MEBLOCK_S3_I2C0_SDA,
              int8_t scl = MEBLOCK_S3_I2C0_SCL,
              uint8_t addr = 0x38,
              TwoWire &wire = Wire);

  // Gọi trong USER_SETUP
  bool begin(uint32_t freq = 100000UL);

  // Gọi mỗi lần cần cập nhật dữ liệu
  bool read();

  // Lấy lại giá trị lần đọc gần nhất
  float temperature() const { return _t; }
  float humidity()    const { return _h; }
  bool  lastOk()      const { return _lastOk; }

private:
  int8_t  _sda;
  int8_t  _scl;
  uint8_t _addr;
  TwoWire *_wire;

  Adafruit_AHTX0 _aht;

  bool  _lastOk;
  float _t;
  float _h;
};

// ========== DHT11 (GPIO) ==========
class Dht11Sensor {
public:
  explicit Dht11Sensor(uint8_t pin = MEBLOCK_S3_DHT11_PIN);

  bool begin();   // Gọi trong USER_SETUP
  bool read();    // Gọi để cập nhật

  float temperature() const { return _t; }
  float humidity()    const { return _h; }
  bool  lastOk()      const { return _lastOk; }

private:
  uint8_t _pin;
  DHT     _dht;

  bool  _lastOk;
  float _t;
  float _h;
};
