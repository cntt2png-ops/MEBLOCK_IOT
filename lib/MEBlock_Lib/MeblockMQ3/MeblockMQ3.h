#pragma once
#include <Arduino.h>
#include <math.h>

/*
  MeblockMQ - MQ3 (alcohol) port từ mq3.py
  Mapping Blockly:
    - MQ3 var(pin)
    - var.begin()
    - var.calibrate()
    - var.setSlowMode(true/false)
    - var.readAlcoholMgL() / var.readAlcoholPpm()
*/

class BaseMQ {
public:
  static const int STRATEGY_FAST = 0;
  static const int STRATEGY_SLOW = 1;

  BaseMQ(int pinData,
         float boardResistance = 10.0f,
         float baseVoltage = 3.3f,
         int measuringStrategy = STRATEGY_FAST,
         bool invert = false);

  virtual ~BaseMQ() {}

  void begin();

  // Map cho block "Chế độ đọc"
  void setSlowMode(bool slow);

  // (tuỳ chọn) tương đương set_invert
  void setInvert(bool invert);

  // (tuỳ chọn) tương đương set_calib
  void setCalibration(float ro);

  // Map cho block "Hiệu chỉnh"
  float calibrate(uint32_t durationMs = 1500);

protected:
  virtual float getRoInCleanAir() const { return 1.0f; }

  float readScaled(float m, float b);

  int   _pinData;
  float _boardResistance;
  float _baseVoltage;
  int   _measuringStrategy;
  bool  _invert;

  int   _samplesFast = 5;
  int   _samplesSlow = 20;

  float _ro = -1.0f;
  bool  _inited = false;

protected:
  int   readAdcOnce();
  float readAdcAvg();
  float calcRs(float adcValue);
  float rsRoRatio();
};

class MQ3 : public BaseMQ {
public:
  static constexpr float MQ3_RO_IN_CLEAN_AIR = 60.0f;

  explicit MQ3(int pinData = 4,
               float boardResistance = 10.0f,
               float baseVoltage = 3.3f,
               bool invert = false);

  // Map cho block "Đọc nồng độ" với TYPE
  float readAlcoholMgL();
  float readAlcoholPpm();

protected:
  float getRoInCleanAir() const override { return MQ3_RO_IN_CLEAN_AIR; }
};
