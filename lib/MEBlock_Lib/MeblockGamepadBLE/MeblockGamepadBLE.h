#pragma once
#include <Arduino.h>

// ===== Auto BLE stack detect =====
// - If you want to force classic BLE (BLEDevice.h): define MEBLOCK_FORCE_BLEDEVICE
// - If you want to force NimBLE (NimBLEDevice.h): define MEBLOCK_FORCE_NIMBLE
#if defined(MEBLOCK_FORCE_BLEDEVICE)
  #define MEBLOCK_USE_NIMBLE 0
#elif defined(MEBLOCK_FORCE_NIMBLE)
  #define MEBLOCK_USE_NIMBLE 1
#elif __has_include(<NimBLEDevice.h>)
  #define MEBLOCK_USE_NIMBLE 1
#elif __has_include(<BLEDevice.h>)
  #define MEBLOCK_USE_NIMBLE 0
#else
  #error "Need NimBLEDevice.h or BLEDevice.h"
#endif

#if MEBLOCK_USE_NIMBLE
  #include <NimBLEDevice.h>
#else
  #include <BLEDevice.h>
  #include <BLEServer.h>
  #include <BLEUtils.h>
  #include <BLE2902.h>
#endif

class MeblockGamepadBLE {
public:
  bool begin(const char* name, int deadzone = 6);
  void end();

  bool isConnected() const { return _connected; }
  bool waitConnected(uint32_t timeoutMs = 0, uint32_t pollMs = 50);

  // Parse up to maxLines frames from RX buffer.
  // Return number of parsed "G," frames.
  int update(uint8_t maxLines = 3);

  // reads: return 0/1
  int readButton(char key) const;            // 'T','O','S','X'
  int readDpad(char dir) const;              // 'U','D','L','R'
  int readShoulder(const char* sh) const;    // "L1" / "R1"

  // joystick: side 'L'/'R'
  int joyX(char side) const;                 // -100..100
  int joyY(char side) const;                 // -100..100
  int joyDistance(char side) const;          // 0..100
  int joyAngle(char side) const;             // 0..359

  // ms since last valid G-frame
  uint32_t ageMs() const;

private:
  // ===== BLE UUIDs (NUS) =====
  static constexpr const char* NUS_SERVICE = "6e400001-b5a3-f393-e0a9-e50e24dcca9e";
  static constexpr const char* NUS_RX_CHAR = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"; // write
  static constexpr const char* NUS_TX_CHAR = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"; // notify

#if MEBLOCK_USE_NIMBLE
  NimBLEServer* _server = nullptr;
  NimBLEService* _svc = nullptr;
  NimBLECharacteristic* _rx = nullptr;
  NimBLECharacteristic* _tx = nullptr;
#else
  BLEServer* _server = nullptr;
  BLEService* _svc = nullptr;
  BLECharacteristic* _rx = nullptr;
  BLECharacteristic* _tx = nullptr;
#endif

  volatile bool _connected = false;
  int _deadzone = 6;

  // ===== state =====
  uint8_t _btn[4]  = {0,0,0,0};   // X O S T (frame order)
  uint8_t _dpad[4] = {0,0,0,0};   // U D L R
  uint8_t _sh[2]   = {0,0};       // L1 R1
  int16_t _jlx = 0, _jly = 0, _jrx = 0, _jry = 0;

  // ===== RX ring buffer =====
  static constexpr uint16_t RX_BUF_SZ = 2048;
  char _rxBuf[RX_BUF_SZ];
  volatile uint16_t _head = 0;
  volatile uint16_t _tail = 0;

#if defined(ESP32)
  portMUX_TYPE _mux = portMUX_INITIALIZER_UNLOCKED;
  inline void _lock()   { portENTER_CRITICAL(&_mux); }
  inline void _unlock() { portEXIT_CRITICAL(&_mux);  }
#else
  inline void _lock()   {}
  inline void _unlock() {}
#endif

  // current partial line accumulation
  char _line[96];
  uint8_t _lineLen = 0;

  uint32_t _lastSnapshotMs = 0;

  inline int _dz(int v) const {
    v = (v < -100) ? -100 : (v > 100 ? 100 : v);
    if (_deadzone && abs(v) < _deadzone) return 0;
    return v;
  }

  void _pushBytes(const uint8_t* data, size_t len);
  bool _popByte(char& out);
  bool _readLine(char* out, size_t outSize);

  bool _handleLine(const char* line);
  bool _parseSnapshot(const char* line); // "G,..."

  void _sendNotify(const char* s);

  // callbacks
#if MEBLOCK_USE_NIMBLE
  class _ServerCB : public NimBLEServerCallbacks {
  public:
    explicit _ServerCB(MeblockGamepadBLE* p): _p(p) {}
    void onConnect(NimBLEServer*) override { _p->_connected = true; }
    void onDisconnect(NimBLEServer*) override {
      _p->_connected = false;
      NimBLEDevice::getAdvertising()->start();
    }
  private:
    MeblockGamepadBLE* _p;
  };

  class _RxCB : public NimBLECharacteristicCallbacks {
  public:
    explicit _RxCB(MeblockGamepadBLE* p): _p(p) {}
    void onWrite(NimBLECharacteristic* c) override {
      auto v = c->getValue();               // std::string
      size_t n = v.length();
      if (n) _p->_pushBytes((const uint8_t*)v.c_str(), n);
    }
  private:
    MeblockGamepadBLE* _p;
  };
#else
  class _ServerCB : public BLEServerCallbacks {
  public:
    explicit _ServerCB(MeblockGamepadBLE* p): _p(p) {}
    void onConnect(BLEServer*) override { _p->_connected = true; }
    void onDisconnect(BLEServer*) override {
      _p->_connected = false;
      BLEDevice::startAdvertising();
    }
  private:
    MeblockGamepadBLE* _p;
  };

  class _RxCB : public BLECharacteristicCallbacks {
  public:
    explicit _RxCB(MeblockGamepadBLE* p): _p(p) {}
    void onWrite(BLECharacteristic* c) override {
      auto v = c->getValue();               // ESP32 BLE lib: often returns String
      size_t n = v.length();                // works for both String and std::string
      if (n) _p->_pushBytes((const uint8_t*)v.c_str(), n);
    }
  private:
    MeblockGamepadBLE* _p;
  };
#endif
};
