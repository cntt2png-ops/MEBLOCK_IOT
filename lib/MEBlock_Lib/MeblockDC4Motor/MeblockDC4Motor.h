#pragma once
#include <Arduino.h>
#include <Wire.h>

// 4 DC motors via PCA9685 -> TB6612FNG (2 chips)
// speed: -100..100
// stop(brake=false): coast
// stop(brake=true): short brake (IN1=IN2=1, PWM=100%)
class MeblockDC4Motor {
public:
  explicit MeblockDC4Motor(uint8_t pcaAddr = 0x40);

  // For ESP32 you can pass SDA/SCL. For AVR you can ignore pins (use default Wire.begin()).
  bool begin(TwoWire& w = Wire, int sda = -1, int scl = -1, uint32_t i2cFreq = 400000, uint16_t pwmFreq = 1000);

  void set(uint8_t motor, int16_t speed);       // motor: 1..4, speed: -100..100
  void stop(uint8_t motor, bool brake=false);
  void stopAll(bool brake=false);

  void invert(uint8_t motor, bool enable=true); // reverse direction for a motor

  // Optional: change PWM frequency later
  void setPwmFreq(uint16_t pwmFreq);

private:
  // PCA9685 low-level
  void write8(uint8_t reg, uint8_t val);
  uint8_t read8(uint8_t reg);
  void setPWM(uint8_t ch, uint16_t on, uint16_t off);
  void setDuty(uint8_t ch, uint16_t duty0_4095);
  void setLevel(uint8_t ch, bool high);

  void setDir(uint8_t in1, uint8_t in2, bool forward);

private:
  TwoWire* _wire;
  uint8_t _addr;
  bool _inited;
  bool _inv[5]; // 1..4 used

  struct Map3 { uint8_t pwm, in1, in2; };
  Map3 _m[5];   // 1..4 used
};
