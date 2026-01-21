#include "MeblockUltrasonic.h"

Hcsr04Sensor::Hcsr04Sensor(int8_t triggerPin,
                           int8_t echoPin,
                           unsigned long echoTimeoutUs,
                           bool triggerActiveHigh)
    : _trigPin(triggerPin),
      _echoPin(echoPin),
      _echoTimeoutUs(echoTimeoutUs),
      _triggerActiveHigh(triggerActiveHigh),
      _sampleCount(0)
{
    for (uint8_t i = 0; i < _maxSamples; ++i) {
        _samples[i] = 0.0f;
    }
}

void Hcsr04Sensor::begin() {
    pinMode(_trigPin, OUTPUT);
    pinMode(_echoPin, INPUT);

    // Mức idle ngược với mức active
    digitalWrite(_trigPin, _triggerActiveHigh ? LOW : HIGH);
}

unsigned long Hcsr04Sensor::_sendPulseAndWait() {
    // Đưa về trạng thái idle
    digitalWrite(_trigPin, _triggerActiveHigh ? LOW : HIGH);
    delayMicroseconds(5);

    // Gửi xung 10us
    digitalWrite(_trigPin, _triggerActiveHigh ? HIGH : LOW);
    delayMicroseconds(10);
    digitalWrite(_trigPin, _triggerActiveHigh ? LOW : HIGH);

    // Chờ echo lên HIGH, có timeout
    unsigned long duration = pulseIn(_echoPin, HIGH, _echoTimeoutUs);

    if (duration == 0) {
        // Timeout → coi như ngoài vùng đo
        duration = _echoTimeoutUs;
    }

    return duration;
}

float Hcsr04Sensor::distanceCm(bool filter) {
    unsigned long pulseTime = _sendPulseAndWait();

    // Đổi sang cm: (time/2) / 29.1
    float cms = (pulseTime / 2.0f) / 29.1f;
    if (cms < 0.0f || cms > _maxDistanceCm) {
        cms = _maxDistanceCm;
    }

    if (!filter) {
        return cms;
    }

    // Lọc: trung bình trượt 5 mẫu gần nhất
    if (_sampleCount < _maxSamples) {
        _samples[_sampleCount++] = cms;
    } else {
        // dịch mảng sang trái
        for (uint8_t i = 1; i < _maxSamples; ++i) {
            _samples[i - 1] = _samples[i];
        }
        _samples[_maxSamples - 1] = cms;
    }

    float sum = 0.0f;
    for (uint8_t i = 0; i < _sampleCount; ++i) {
        sum += _samples[i];
    }
    float avg = sum / (float)_sampleCount;

    // Làm tròn 0.1 cm (không cần math.h)
    float scaled = avg * 10.0f;
    long rounded = (long)(scaled + 0.5f);
    return (float)rounded / 10.0f;
}

uint16_t Hcsr04Sensor::distanceMm(bool filter) {
    float cm = distanceCm(filter);
    if (cm < 0.0f) cm = 0.0f;
    return (uint16_t)(cm * 10.0f);
}
