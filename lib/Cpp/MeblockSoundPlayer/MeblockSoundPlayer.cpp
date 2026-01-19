#include "MeblockSoundPlayer.h"

SoundPlayer::SoundPlayer(int txPin, int rxPin, uint32_t baudrate)
: _tx(txPin), _rx(rxPin), _baud(baudrate) {}

void SoundPlayer::begin(int uartNum) {
  _uartNum = uartNum;

#if defined(ARDUINO_ARCH_ESP32)
  // ESP32: có Serial1/Serial2... nhưng để chắc chắn, dùng HardwareSerial(uartNum)
  static HardwareSerial hs1(1);
  static HardwareSerial hs2(2);

  if (_uartNum == 2) _ser = &hs2;
  else _ser = &hs1;

  _ser->begin(_baud, SERIAL_8N1, _rx, _tx);
#else
  // fallback: dùng Serial1 nếu có
  _ser = &Serial1;
  _ser->begin(_baud);
#endif

  delay(80);     // giống python sleep_ms(80)
  stop();        // giống python: stop rồi set volume default
  setVolume(20);
}

uint8_t SoundPlayer::clampU8(int v, int lo, int hi) {
  if (v < lo) return (uint8_t)lo;
  if (v > hi) return (uint8_t)hi;
  return (uint8_t)v;
}

uint16_t SoundPlayer::clampU16(int v, int lo, int hi) {
  if (v < lo) return (uint16_t)lo;
  if (v > hi) return (uint16_t)hi;
  return (uint16_t)v;
}

uint8_t SoundPlayer::sm(const uint8_t* payload, size_t n) {
  uint8_t s = 0;
  for (size_t i = 0; i < n; i++) {
    s = (uint8_t)((s + payload[i]) & 0xFF);
  }
  return (uint8_t)((0xAA + s) & 0xFF);
}

void SoundPlayer::writeBytes(const uint8_t* data, size_t n) {
  if (!_ser) return;
  _ser->write(data, n);
}

void SoundPlayer::play() {
  uint8_t cmd[4] = {0xAA, 0x02, 0x00, 0xAC};
  writeBytes(cmd, sizeof(cmd));
}

void SoundPlayer::pause() {
  uint8_t cmd[4] = {0xAA, 0x03, 0x00, 0xAD};
  writeBytes(cmd, sizeof(cmd));
}

void SoundPlayer::stop() {
  uint8_t cmd[4] = {0xAA, 0x04, 0x00, 0xAE};
  writeBytes(cmd, sizeof(cmd));
}

void SoundPlayer::setVolume(uint8_t level) {
  uint8_t lv = clampU8((int)level, 0, 30);

  // python:
  // [0xAA, 0x13, 0x01, level, checksum]
  uint8_t cmd[5];
  cmd[0] = 0xAA;
  cmd[1] = 0x13;
  cmd[2] = 0x01;
  cmd[3] = lv;
  cmd[4] = sm(&cmd[1], 3); // payload = [cmd1, cmd2, cmd3]
  writeBytes(cmd, sizeof(cmd));
}

void SoundPlayer::playTrack(uint16_t trackId) {
  uint16_t id = clampU16((int)trackId, 1, 65535);
  uint8_t hb = (uint8_t)((id >> 8) & 0xFF);
  uint8_t lb = (uint8_t)(id & 0xFF);

  // python:
  // [0xAA, 0x07, 0x02, hb, lb, checksum]
  uint8_t cmd[6];
  cmd[0] = 0xAA;
  cmd[1] = 0x07;
  cmd[2] = 0x02;
  cmd[3] = hb;
  cmd[4] = lb;
  cmd[5] = sm(&cmd[1], 4); // payload = [cmd1,cmd2,cmd3,cmd4]
  writeBytes(cmd, sizeof(cmd));
}
