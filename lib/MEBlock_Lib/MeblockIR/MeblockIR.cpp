#include "MeblockIR.h"

#if !defined(ARDUINO_ARCH_ESP32)
IRRx* IRRx::_singleton = nullptr;
#endif

IRRx::IRRx(uint8_t pin) : _pin(pin) {}

void IRRx::begin() {
  pinMode(_pin, INPUT);
  clear();

#if defined(ARDUINO_ARCH_ESP32)
  // ESP32 family (ESP32/ESP32C3/ESP32S3...)
  attachInterruptArg((int)_pin, &IRRx::isrThunk, this, CHANGE);
#else
  // Fallback cho core không có attachInterruptArg: chỉ hỗ trợ 1 instance
  _singleton = this;
  attachInterrupt(digitalPinToInterrupt(_pin), &IRRx::isrThunkNoArg, CHANGE);
#endif
}

void IRRx::end() {
#if defined(ARDUINO_ARCH_ESP32)
  detachInterrupt((int)_pin);
#else
  detachInterrupt(digitalPinToInterrupt(_pin));
#endif
}

void IRRx::clear() {
  _key = -1;
  _hasNew = false;
}

bool IRRx::available() {
  process();
  return (bool)_hasNew;
}

int IRRx::getCode() {
  process();
  return _key;
}

int IRRx::readCode() {
  process();
  if (!_hasNew) return -1;
  _hasNew = false;
  return _key;
}

bool IRRx::isPressed(uint8_t key) {
  process();
  return (_key >= 0) && ((uint8_t)_key == key);
}

void IRRx::process() {
  if (_edge < 3) return; // cần tối thiểu t[1], t[2]
  const uint32_t now = micros();
  const uint32_t t0  = _t0;
  if (diffUs(now, t0) < TBLOCK_US) return;

  uint8_t edges = 0;
  uint32_t t[IR_EDGES + 1];

  noInterrupts();
  edges = _edge;
  if (edges > IR_EDGES) edges = IR_EDGES;
  for (uint8_t i = 0; i < edges; i++) {
    t[i] = _times[i];
  }
  _edge = 0;
  interrupts();

  decode(t, edges);
}

void IRRx::decode(const uint32_t* t, uint8_t edges) {
  if (edges > IR_EDGES) return;
  if (edges < 3) return;

  // width = t[2] - t[1] (đúng như MicroPython)
  const uint32_t width = diffUs(t[2], t[1]);

  // Full frame
  if (width > 2500) {
    uint32_t val = 0;

    // for edge in range(3, 68-2, 2) => 3..65
    for (uint8_t e = 3; e < (IR_EDGES - 2); e += 2) {
      val >>= 1;
      if (diffUs(t[e + 1], t[e]) > 1120) {
        val |= 0x80000000UL;
      }
    }

    uint16_t addr = (uint16_t)(val & 0xFF);
    const uint8_t cmd = (uint8_t)((val >> 16) & 0xFF);

    // check complement address (giống MicroPython)
    if ((uint8_t)addr != (uint8_t)(((val >> 8) ^ 0xFF) & 0xFF)) {
      addr |= (uint16_t)(val & 0xFF00);
    }

    _addr = addr;
    _key = (int)cmd;
    _lastKey = _key;
    _hasNew = true;
    return;
  }

  // Repeat frame: width ~ 2250us (giống MicroPython: if width > 110)
  if (width > 110) {
    if (_lastKey >= 0) {
      _key = _lastKey;
      _hasNew = true;
    }
    return;
  }

  // Bad start -> ignore
}

#if defined(ARDUINO_ARCH_ESP32)
void IRAM_ATTR IRRx::isrThunk(void* arg) {
  IRRx* self = reinterpret_cast<IRRx*>(arg);
  if (self) self->handleEdge();
}
#else
void IRAM_ATTR IRRx::isrThunkNoArg() {
  if (_singleton) _singleton->handleEdge();
}
#endif

void IRAM_ATTR IRRx::handleEdge() {
  const uint32_t t = micros();

#if defined(ARDUINO_ARCH_ESP32)
  portENTER_CRITICAL_ISR(&_mux);
#endif

  if (_edge <= IR_EDGES) {
    if (_edge == 0) _t0 = t;
    _times[_edge] = t;
    _edge++;
  }

#if defined(ARDUINO_ARCH_ESP32)
  portEXIT_CRITICAL_ISR(&_mux);
#endif
}
