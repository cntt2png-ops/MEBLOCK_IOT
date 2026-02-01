#include "MeblockDC4Motor.h"

// PCA9685 registers
static const uint8_t MODE1      = 0x00;
static const uint8_t MODE2      = 0x01;
static const uint8_t PRESCALE   = 0xFE;
static const uint8_t LED0_ON_L  = 0x06;

static const uint8_t RESTART    = 0x80;
static const uint8_t SLEEP      = 0x10;
static const uint8_t AI         = 0x20; // auto increment
static const uint8_t OUTDRV     = 0x04; // MODE2 totem pole

MeblockDC4Motor::MeblockDC4Motor(uint8_t pcaAddr)
: _wire(&Wire), _addr(pcaAddr), _inited(false) {
  for (int i=0;i<5;i++) _inv[i]=false;

  // ==== MAP theo mạch PCA9685 -> TB6612 ====
  // M1: LED0=PWMA, LED2=IN1,  LED1=IN2
  _m[1] = {0, 2, 1};

  // M2: LED3=PWMB, LED4=INB1, LED5=INB2
  _m[2] = {3, 4, 5};

  // M3: theo sơ đồ: LED6=PWMA3, LED7=IN32, LED8=IN31
  // (Nếu bạn đã xác nhận chiều thuận đúng với map khác thì đổi ở đây cho khớp thực tế.)
  _m[3] = {6, 8, 7}; // IN1=LED8(IN31), IN2=LED7(IN32)

  // M4: sơ đồ: LED11=PWMB4, LED9=INB41, LED10=INB42
  _m[4] = {11, 9, 10};
}

bool MeblockDC4Motor::begin(TwoWire& w, int sda, int scl, uint32_t i2cFreq, uint16_t pwmFreq) {
  _wire = &w;

  // ESP32 supports Wire.begin(sda,scl,freq). AVR ignores pins.
  #if defined(ESP32)
    if (sda >= 0 && scl >= 0) _wire->begin((int)sda, (int)scl, i2cFreq);
    else _wire->begin();
    _wire->setClock(i2cFreq);
  #else
    (void)sda; (void)scl; (void)i2cFreq;
    _wire->begin();
  #endif

  // Init PCA9685
  write8(MODE1, AI);       // auto-increment, normal
  write8(MODE2, OUTDRV);   // totem pole
  delay(10);

  setPwmFreq(pwmFreq);

  stopAll(false);
  _inited = true;
  return true;
}

void MeblockDC4Motor::setPwmFreq(uint16_t pwmFreq) {
  // prescale = round(25MHz/(4096*freq)) - 1
  if (pwmFreq < 24) pwmFreq = 24;
  if (pwmFreq > 1526) pwmFreq = 1526;

  float prescaleval = 25000000.0f;
  prescaleval /= 4096.0f;
  prescaleval /= (float)pwmFreq;
  prescaleval -= 1.0f;

  uint8_t prescale = (uint8_t)(prescaleval + 0.5f);
  if (prescale < 3) prescale = 3;

  uint8_t oldmode = read8(MODE1);
  uint8_t sleepmode = (oldmode & ~RESTART) | SLEEP;

  write8(MODE1, sleepmode);
  write8(PRESCALE, prescale);
  write8(MODE1, oldmode);
  delay(5);
  write8(MODE1, oldmode | RESTART | AI);
}

void MeblockDC4Motor::invert(uint8_t motor, bool enable) {
  if (motor < 1 || motor > 4) return;
  _inv[motor] = enable;
}

void MeblockDC4Motor::set(uint8_t motor, int16_t speed) {
  if (!_inited) return;
  if (motor < 1 || motor > 4) return;

  speed = constrain(speed, (int16_t)-100, (int16_t)100);

  const auto &mm = _m[motor];

  if (speed == 0) {
    stop(motor, false);
    return;
  }

  bool forward = (speed > 0);
  if (_inv[motor]) forward = !forward;

  uint16_t duty = (uint16_t)((abs(speed) * 4095L) / 100L);

  // set direction then PWM
  setDir(mm.in1, mm.in2, forward);
  setDuty(mm.pwm, duty);
}

void MeblockDC4Motor::stop(uint8_t motor, bool brake) {
  if (!_inited) return;
  if (motor < 1 || motor > 4) return;

  const auto &mm = _m[motor];

  if (brake) {
    // short brake: IN1=IN2=1, PWM=100%
    setLevel(mm.in1, true);
    setLevel(mm.in2, true);
    setDuty(mm.pwm, 4095);
  } else {
    // coast: IN1=IN2=0, PWM=0
    setLevel(mm.in1, false);
    setLevel(mm.in2, false);
    setDuty(mm.pwm, 0);
  }
}

void MeblockDC4Motor::stopAll(bool brake) {
  for (uint8_t m=1; m<=4; m++) stop(m, brake);
}

// ---------------- PCA9685 helpers ----------------
void MeblockDC4Motor::write8(uint8_t reg, uint8_t val) {
  _wire->beginTransmission(_addr);
  _wire->write(reg);
  _wire->write(val);
  _wire->endTransmission();
}

uint8_t MeblockDC4Motor::read8(uint8_t reg) {
  _wire->beginTransmission(_addr);
  _wire->write(reg);
  _wire->endTransmission(false);
  _wire->requestFrom((int)_addr, 1);
  return _wire->available() ? _wire->read() : 0;
}

void MeblockDC4Motor::setPWM(uint8_t ch, uint16_t on, uint16_t off) {
  uint8_t reg = LED0_ON_L + 4 * ch;
  _wire->beginTransmission(_addr);
  _wire->write(reg);
  _wire->write(on & 0xFF);
  _wire->write((on >> 8) & 0xFF);
  _wire->write(off & 0xFF);
  _wire->write((off >> 8) & 0xFF);
  _wire->endTransmission();
}

void MeblockDC4Motor::setLevel(uint8_t ch, bool high) {
  // full ON/OFF by bit 12
  if (high) setPWM(ch, 4096, 0);
  else      setPWM(ch, 0, 4096);
}

void MeblockDC4Motor::setDuty(uint8_t ch, uint16_t duty0_4095) {
  if (duty0_4095 <= 0) {
    setLevel(ch, false);
  } else if (duty0_4095 >= 4095) {
    setLevel(ch, true);
  } else {
    setPWM(ch, 0, duty0_4095);
  }
}

void MeblockDC4Motor::setDir(uint8_t in1, uint8_t in2, bool forward) {
  if (forward) {
    setLevel(in1, true);
    setLevel(in2, false);
  } else {
    setLevel(in1, false);
    setLevel(in2, true);
  }
}
