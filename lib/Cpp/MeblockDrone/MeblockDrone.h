#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include <math.h>

// Cấu hình truyền vào begin() để main chỉ set WiFi/IP + vài tham số bay mặc định.
// Lưu ý: pins (motor/I2C/...) được ẩn trong thư viện để giữ logic drone gốc.
struct MeblockDroneConfig {
  // SoftAP Web UI
  const char* apSsid = "DragonFly Web Tuning";
  const char* apPass = "12345678";
  IPAddress   apIP   = IPAddress(192, 168, 4, 1);

  // ---- Default tuning overrides (tuỳ chọn) ----
  // Nếu để -1 hoặc NAN thì dùng mặc định trong firmware (hoặc giá trị đã lưu trong Preferences).
  // Load tuning: ưu tiên giá trị đã lưu (nếu có), sau đó mới fallback về mặc định.
  int   baseSpeed = -1;                 // ví dụ: 480
  int   integralLimit = -1;             // anti wind-up (nếu firmware dùng int)
  float range_altitude = NAN;           // range độ cao (m)
  float maxRateChange  = NAN;           // slew (m/s)

  // Speed tuning
  float maxAngleDeg     = NAN;
  float maxYawRateDegS  = NAN;
  float maxAltRateMps   = NAN;
  float rpResponse      = NAN;
  float yawResponse     = NAN;
  float altResponse     = NAN;
  float rpExpo          = NAN;
  float yawExpo         = NAN;
  float altExpo         = NAN;

  // Feed-forward / load
  float kff_roll     = NAN;
  float kff_pitch    = NAN;
  float kff_yaw      = NAN;
  float kff_altitude = NAN;
};


class MeblockDrone {
public:
  void begin(const MeblockDroneConfig& cfg = MeblockDroneConfig());
  void update_and_fly();
};
