#pragma once
#include <Arduino.h>

/*
  Meblockrgb - WS2812 (Module 4 LED RGB)
  Mapping Blockly:
    INCLUDES : #include <Meblockrgb.h>
    CODE_STMT: _rgb4_set(PIN, OPTION, _rgb4_hex("#RRGGBB"));

  OPTION:
    0    -> tất cả LED
    1..4 -> LED số 1..4
*/

uint32_t _rgb4_hex(const char* color);
void _rgb4_set(int pin, int option, uint32_t rgb);
void _rgb4_clear(int pin);
