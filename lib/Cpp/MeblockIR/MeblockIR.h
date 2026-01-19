#pragma once
#include <Arduino.h>

/*
  MeblockIR - IRRx (NEC-like) decoder
  API để map Blockly:
    - IRRx(pin)
    - begin()
    - available()
    - isPressed(IR_REMOTE_X)
    - readCode()
    - clear()
*/

#ifndef IRAM_ATTR
#define IRAM_ATTR
#endif

// ===== Remote key codes (khớp file MicroPython IR) =====
#define IR_REMOTE_A      0x45
#define IR_REMOTE_B      0x46
#define IR_REMOTE_C      0x47
#define IR_REMOTE_D      0x44
#define IR_REMOTE_E      0x43
#define IR_REMOTE_F      0x0D
#define IR_REMOTE_UP     0x40
#define IR_REMOTE_DOWN   0x19
#define IR_REMOTE_LEFT   0x07
#define IR_REMOTE_RIGHT  0x09
#define IR_REMOTE_SETUP  0x15
#define IR_REMOTE_0      0x16
#define IR_REMOTE_1      0x0C
#define IR_REMOTE_2      0x18
#define IR_REMOTE_3      0x5E
#define IR_REMOTE_4      0x08
#define IR_REMOTE_5      0x1C
#define IR_REMOTE_6      0x5A
#define IR_REMOTE_7      0x42
#define IR_REMOTE_8      0x52
#define IR_REMOTE_9      0x4A

class IRRx {
public:
  explicit IRRx(uint8_t pin = 5);

  // Map: init block sẽ gọi begin() trong setup
  void begin();
  void end();

  // Map reporter
  bool available();
  bool isPressed(uint8_t key);
  int  readCode();   // trả -1 nếu chưa có mã mới
  int  getCode();    // đọc code hiện tại (không clear new-flag)

  // Map statement
  void clear();

  // (tuỳ chọn)
  uint16_t address() const { return _addr; }

private:
  // Giữ giống MicroPython
  static const uint8_t  IR_EDGES   = 68;
  static const uint32_t TBLOCK_US  = 80000UL;  // 80ms

#if defined(ARDUINO_ARCH_ESP32)
  static void IRAM_ATTR isrThunk(void* arg);
#else
  static void IRAM_ATTR isrThunkNoArg();
  static IRRx* _singleton;
#endif

  void IRAM_ATTR handleEdge();
  void process();
  void decode(const uint32_t* t, uint8_t edges);

  inline uint32_t diffUs(uint32_t a, uint32_t b) const { return (uint32_t)(a - b); }

private:
  uint8_t _pin;

#if defined(ARDUINO_ARCH_ESP32)
  portMUX_TYPE _mux = portMUX_INITIALIZER_UNLOCKED;
#endif

  volatile uint8_t  _edge = 0;
  volatile uint32_t _times[IR_EDGES + 1];
  volatile uint32_t _t0 = 0; // time of first edge

  volatile bool _hasNew = false;

  int      _key = -1;
  int      _lastKey = -1;
  uint16_t _addr = 0;
};
