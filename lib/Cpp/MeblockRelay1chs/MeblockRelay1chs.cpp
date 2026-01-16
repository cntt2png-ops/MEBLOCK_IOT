#include "MeblockRelay1chs.h"

MeblockRelay::MeblockRelay(uint8_t pin, bool activeHigh)
: _pin(pin), _activeHigh(activeHigh) {}

void MeblockRelay::begin() {
  pinMode(_pin, OUTPUT);
  off(); // đảm bảo relay tắt khi khởi động
}

void MeblockRelay::on() {
  digitalWrite(_pin, levelOn());
}

void MeblockRelay::off() {
  digitalWrite(_pin, levelOff());
}
