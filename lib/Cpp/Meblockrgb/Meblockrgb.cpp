#include "Meblockrgb.h"
#include <ctype.h>

#if defined(ARDUINO_ARCH_ESP32)
  #include "driver/rmt.h"   // vẫn dùng legacy, warning deprecated có thể bỏ qua
#endif

// ===== Hex parser =====
static inline int _hexval(char ch) {
  if (ch >= '0' && ch <= '9') return ch - '0';
  ch = (char)tolower((unsigned char)ch);
  if (ch >= 'a' && ch <= 'f') return 10 + (ch - 'a');
  return -1;
}

uint32_t _rgb4_hex(const char* color) {
  if (!color) return 0;

  while (*color && isspace((unsigned char)*color)) color++;
  if (*color == '#') color++;
  if (color[0] == '0' && (color[1] == 'x' || color[1] == 'X')) color += 2;

  char s[8] = {0};
  int n = 0;
  while (*color && n < 6) {
    if (!isspace((unsigned char)*color)) s[n++] = *color;
    color++;
  }
  s[n] = 0;

  // short RGB => RRGGBB
  if (n == 3) {
    char t[7];
    t[0] = s[0]; t[1] = s[0];
    t[2] = s[1]; t[3] = s[1];
    t[4] = s[2]; t[5] = s[2];
    t[6] = 0;
    return _rgb4_hex(t);
  }

  if (n != 6) return 0;

  uint32_t v = 0;
  for (int i = 0; i < 6; i++) {
    int hv = _hexval(s[i]);
    if (hv < 0) return 0;
    v = (v << 4) | (uint32_t)hv;
  }
  return v; // 0xRRGGBB
}

// ✅ đổi tên để tránh đụng macro (_B)
static inline uint8_t _rgbR(uint32_t c) { return (uint8_t)((c >> 16) & 0xFF); }
static inline uint8_t _rgbG(uint32_t c) { return (uint8_t)((c >>  8) & 0xFF); }
static inline uint8_t _rgbB(uint32_t c) { return (uint8_t)( c        & 0xFF); }

#if defined(ARDUINO_ARCH_ESP32)

// ===== Internal cache (giống Python cache theo pin) =====
struct _rgb4_entry_t {
  int pin = -1;
  bool inited = false;
  rmt_channel_t ch = RMT_CHANNEL_0;
  uint32_t buf[4] = {0, 0, 0, 0}; // 0xRRGGBB
};

static _rgb4_entry_t _rgb4_cache[4];
static int _next_ch = 0;

static bool _rgb4_init_rmt(_rgb4_entry_t* e) {
  rmt_config_t config;
  config.rmt_mode = RMT_MODE_TX;
  config.channel = e->ch;
  config.gpio_num = (gpio_num_t)e->pin;
  config.mem_block_num = 1;

  // 80MHz / 8 = 10MHz => 0.1us/tick
  config.clk_div = 8;

  config.tx_config.loop_en = false;
  config.tx_config.carrier_en = false;
  config.tx_config.idle_output_en = true;
  config.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;
  config.tx_config.carrier_level = RMT_CARRIER_LEVEL_HIGH;
  config.tx_config.carrier_duty_percent = 50;
  config.tx_config.carrier_freq_hz = 10000;

  if (rmt_config(&config) != ESP_OK) return false;
  if (rmt_driver_install(e->ch, 0, 0) != ESP_OK) return false;

  e->inited = true;
  return true;
}

static _rgb4_entry_t* _rgb4_get(int pin) {
  for (auto &e : _rgb4_cache) {
    if (e.inited && e.pin == pin) return &e;
  }
  for (auto &e : _rgb4_cache) {
    if (!e.inited) {
      e.pin = pin;
      e.ch  = (rmt_channel_t)_next_ch;
      _next_ch = (_next_ch + 1) % 4;

      if (!_rgb4_init_rmt(&e)) {
        e.pin = -1;
        e.inited = false;
        return nullptr;
      }
      for (int i = 0; i < 4; i++) e.buf[i] = 0;
      return &e;
    }
  }
  return nullptr;
}

// WS2812 timing @10MHz tick (0.1us)
static inline void _ws2812_bit(rmt_item32_t &it, bool one) {
  if (one) {
    it.level0 = 1; it.duration0 = 8;  // 0.8us
    it.level1 = 0; it.duration1 = 4;  // 0.4us
  } else {
    it.level0 = 1; it.duration0 = 4;  // 0.4us
    it.level1 = 0; it.duration1 = 8;  // 0.8us
  }
}

static void _rgb4_show(_rgb4_entry_t* e) {
  // 4 LED * 24 bit = 96 items
  rmt_item32_t items[96];
  int idx = 0;

  for (int p = 0; p < 4; p++) {
    // WS2812 order: GRB
    uint8_t g = _rgbG(e->buf[p]);
    uint8_t r = _rgbR(e->buf[p]);
    uint8_t b = _rgbB(e->buf[p]);

    uint8_t bytes[3] = { g, r, b };

    for (int bi = 0; bi < 3; bi++) {
      for (int bit = 7; bit >= 0; bit--) {
        bool one = (bytes[bi] >> bit) & 0x01;
        _ws2812_bit(items[idx++], one);
      }
    }
  }

  rmt_write_items(e->ch, items, idx, true);
  rmt_wait_tx_done(e->ch, pdMS_TO_TICKS(50));
  delayMicroseconds(80); // latch
}

#endif // ARDUINO_ARCH_ESP32

// ===== Public API =====
void _rgb4_set(int pin, int option, uint32_t rgb) {
#if !defined(ARDUINO_ARCH_ESP32)
  (void)pin; (void)option; (void)rgb;
  return;
#else
  _rgb4_entry_t* e = _rgb4_get(pin);
  if (!e) return;

  int opt = option;
  if (opt < 0) opt = 0;

  if (opt == 0) {
    for (int i = 0; i < 4; i++) e->buf[i] = rgb;
  } else {
    int id = opt - 1;     // 1..4 => 0..3
    if (id < 0) id = 0;
    if (id > 3) id = 3;
    e->buf[id] = rgb;
  }

  _rgb4_show(e);
#endif
}

void _rgb4_clear(int pin) {
  _rgb4_set(pin, 0, 0x000000);
}
