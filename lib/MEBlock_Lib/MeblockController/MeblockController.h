#pragma once
#include <Arduino.h>
#include <WiFi.h>

struct MeblockControllerConfig {
  const char* apSsid = "MEBlock Controller V1";
  const char* apPass = "12345678";
  IPAddress apIP = IPAddress(192, 168, 4, 1);

  // Optional overrides (set -1 to keep firmware default)
  int deadzone = -1;      // joystick deadzone
  int loopDelayMs = -1;   // delay inside update loop (ms)
};

class MeblockController {
public:
  void begin(const MeblockControllerConfig& cfg = MeblockControllerConfig());
  void update();
};
