#pragma once
#include <Arduino.h>

/*
  MeblockSoundPlayer - UART Sound module
  Port từ sound_lib.py

  Mapping Blockly:
    INCLUDES : #include <MeblockSoundPlayer.h>
    GLOBALS  : SoundPlayer {{VAR}}({{TX}}, {{RX}});
    CODE_STMT:
      {{VAR}}.begin();
      {{VAR}}.play(); / pause(); / stop();
      {{VAR}}.playTrack({{VALUE}});
      {{VAR}}.setVolume({{VALUE}});   // 0..30

  Giao thức (giữ theo python):
    base = [0xAA, 0x00, 0x00, 0x00]
    play  : cmd[1]=0x02, cmd[3]=0xAC
    pause : cmd[1]=0x03, cmd[3]=0xAD
    stop  : cmd[1]=0x04, cmd[3]=0xAE
    volume: cmd[1]=0x13, cmd[2]=0x01, cmd[3]=level, append checksum
    track : cmd[1]=0x07, cmd[2]=0x02, cmd[3]=HB, append LB, append checksum
*/

class SoundPlayer {
public:
  // TX/RX: chân UART nối module sound
  explicit SoundPlayer(int txPin = 17, int rxPin = 18, uint32_t baudrate = 9600);

  // giống block init: gọi trong setup
  void begin(int uartNum = 1);

  // Menu actions
  void play();
  void pause();
  void stop();

  // Theo bài / âm lượng
  void playTrack(uint16_t trackId);
  void setVolume(uint8_t level); // 0..30

private:
  int _tx;
  int _rx;
  uint32_t _baud;
  int _uartNum = 1;
  HardwareSerial* _ser = nullptr;

private:
  static uint8_t clampU8(int v, int lo, int hi);
  static uint16_t clampU16(int v, int lo, int hi);

  // checksum theo python: (0xAA + sum(payload)) & 0xFF
  static uint8_t sm(const uint8_t* payload, size_t n);

  void writeBytes(const uint8_t* data, size_t n);
};
