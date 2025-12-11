#pragma once
#include <Arduino.h>
#include <Wire.h>

// Tắt splash của Adafruit để tiết kiệm RAM/flash
#define SH110X_NO_SPLASH
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>

// Mặc định giống preset ESP32-S3 DevKitC-1 bên MicroPython
#ifndef MEBLOCK_S3_I2C0_SDA
#define MEBLOCK_S3_I2C0_SDA 8
#define MEBLOCK_S3_I2C0_SCL 9
#endif

class Oled {
public:
  // SH1106 128x64 I2C, addr 0x3C, SDA/SCL mặc định là 8/9 (ESP32-S3)
  Oled(uint16_t width  = 128,
       uint16_t height = 64,
       int8_t   sda    = MEBLOCK_S3_I2C0_SDA,
       int8_t   scl    = MEBLOCK_S3_I2C0_SCL,
       uint8_t  addr   = 0x3C,
       TwoWire &wire   = Wire);

  // Gọi trong setup(): cấu hình I2C + init màn hình
  bool begin(uint32_t freq = 400000UL);

  // API high-level giống MicroPython
  void clear();                    // fill(0) + show()
  void fill(bool color = true);    // fill toàn bộ buffer, chưa show
  void show();                     // đẩy buffer lên màn

  void text(const String &s,
            int16_t x, int16_t y,
            bool color = true);

  void textWrap(const String &s,
                int16_t x = 0, int16_t y = 0,
                bool color = true,
                uint8_t lineHeight = 8,
                uint8_t charWidth  = 6);

  void powerOff();                 // giả lập tắt: giảm contrast
  void powerOn();                  // bật lại: tăng contrast
  void invert(bool inv);           // đảo màu
  void contrast(uint8_t value);    // set contrast 0..255

  // Helper vẽ giống framebuf: pixel/line/rect...
  void pixel(int16_t x, int16_t y, bool color = true);
  void line(int16_t x0, int16_t y0,
            int16_t x1, int16_t y1,
            bool color = true);
  void hline(int16_t x, int16_t y,
             int16_t w,
             bool color = true);
  void vline(int16_t x, int16_t y,
             int16_t h,
             bool color = true);
  void rect(int16_t x, int16_t y,
            int16_t w, int16_t h,
            bool color = true);
  void fillRect(int16_t x, int16_t y,
                int16_t w, int16_t h,
                bool color = true);

  // Truy cập trực tiếp display gốc nếu cần dùng API Adafruit_GFX
  Adafruit_SH1106G &raw() { return _display; }

  uint16_t width()  const { return _width;  }
  uint16_t height() const { return _height; }

private:
  uint16_t _width;
  uint16_t _height;
  int8_t   _sda;
  int8_t   _scl;
  uint8_t  _addr;
  TwoWire *_wire;

  Adafruit_SH1106G _display;
};
