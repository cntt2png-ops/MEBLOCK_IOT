#pragma once
#include <Arduino.h>

// Default pins for ESP32-S3 DevKitC-1, giống driver MicroPython
#ifndef MEBLOCK_S3_US_TRIG
#define MEBLOCK_S3_US_TRIG 1
#define MEBLOCK_S3_US_ECHO 2
#endif

// Giữ tên macro S3_US_TRIG/ECHO để quen với bản MicroPython
#ifndef S3_US_TRIG
#define S3_US_TRIG MEBLOCK_S3_US_TRIG
#define S3_US_ECHO MEBLOCK_S3_US_ECHO
#endif

class Hcsr04Sensor {
public:
    Hcsr04Sensor(int8_t triggerPin = S3_US_TRIG,
                 int8_t echoPin    = S3_US_ECHO,
                 unsigned long echoTimeoutUs = 500UL * 2UL * 30UL,
                 bool triggerActiveHigh = true);

    // Gọi trong USER_SETUP
    void begin();

    // Đo khoảng cách (cm). Nếu filter=false thì trả về giá trị đo ngay
    float distanceCm(bool filter = true);

    // Đo khoảng cách (mm). Mặc định vẫn filter
    uint16_t distanceMm(bool filter = true);

private:
    int8_t _trigPin;
    int8_t _echoPin;
    unsigned long _echoTimeoutUs;
    bool _triggerActiveHigh;

    static constexpr float _maxDistanceCm = 200.0f;

    static const uint8_t _maxSamples = 5;
    float   _samples[_maxSamples];
    uint8_t _sampleCount;

    unsigned long _sendPulseAndWait();
};
