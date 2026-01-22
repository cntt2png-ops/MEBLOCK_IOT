#pragma once
#include <Arduino.h>
#include <WiFi.h>

// Chỉ dùng để cấu hình SoftAP Web Tuning (giống logic trong file .ino mẫu)
struct MeblockDroneConfig {
  const char* apSsid = "MEBlock Drone V1";
  const char* apPass = "12345678";
  IPAddress   apIP   = IPAddress(192, 168, 4, 1);
};

class MeblockDrone {
public:
  void begin(const MeblockDroneConfig& cfg = MeblockDroneConfig());
  void update_and_fly();
};
