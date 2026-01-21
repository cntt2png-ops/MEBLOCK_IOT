#pragma once
#include <Arduino.h>

/*
  MeblockSoil - Soil Moisture (ADC)
  Mapping Blockly:
    INCLUDES : #include <MeblockSoil.h>
    CODE_EXPR: SoilMoisture(PIN).readPercent()

  Default calib (giống soil.py):
    dry_adc = 3300
    wet_adc = 1400
    samples = 10
    invert  = false
*/

class SoilMoisture {
public:
  explicit SoilMoisture(int pin,
                        int dry_adc = 3300,
                        int wet_adc = 1400,
                        int samples = 10,
                        bool invert = false);

  // đọc % độ ẩm (0..100)
  int readPercent();

  // (tuỳ chọn) nếu sau này bạn muốn thêm block calib
  void setCalib(int dry_adc, int wet_adc);
  void setSamples(int samples);
  void setInvert(bool invert);

private:
  int _pin;
  int _dry;
  int _wet;
  int _samples;
  bool _invert;

private:
  static int clampi(int v, int lo, int hi);
  void ensureAdcConfig();
  int  readAvgRaw();
};
