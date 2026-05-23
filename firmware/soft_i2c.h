#pragma once
#include <Arduino.h>

// ─────────────────────────────────────────────────────────────────────────────
// Usage:
//   SoftI2C bus(SDA_PIN, SCL_PIN, 100000UL);
//
//   void setup() { bus.begin(); }
//
//   void loop()  { bus.tick(); }
//
//   // Enqueue a write; optional callback fires on completion
//   byte buf[] = { 0x00, 0x38 };
//   bus.write(0x3C, buf, 2, myCallback);
//
// Callback signature:  void cb(byte error)
//   error == 0  success
//   error == 1  NAK received from slave
// ─────────────────────────────────────────────────────────────────────────────

#define SOFT_I2C_QUEUE_DEPTH  16
#define SOFT_I2C_MAX_PAYLOAD  32

using I2CCallback = void (*)(byte error);

struct I2CJob {
  byte addr;
  byte data[SOFT_I2C_MAX_PAYLOAD];
  byte len;
  I2CCallback cb;
};

class SoftI2C {
  public:
    SoftI2C(byte sdaPin, byte sclPin, unsigned long freqHz = 100000UL);
    void begin();

    bool write(byte addr, const byte* data, byte len, I2CCallback cb = nullptr);

    bool tick();

    bool idle() const {
      return _state == State::IDLE && _count == 0;
    }

    byte pending() const {
      return _count > 0 ? _count - 1 : 0;
    }

  private:
    void sdaLow()  {
      pinMode(_sda, OUTPUT);
      digitalWrite(_sda, LOW);
    }
    void sdaHigh() {
      pinMode(_sda, INPUT);
    }
    void sclLow()  {
      pinMode(_scl, OUTPUT);
      digitalWrite(_scl, LOW);
    }
    void sclHigh() {
      pinMode(_scl, INPUT);
    }
    bool sdaRead() {
      pinMode(_sda, INPUT);
      return digitalRead(_sda);
    }
    bool sclRead() {
      return digitalRead(_scl);
    }

    byte _sda;
    byte _scl;

    I2CJob  _queue[SOFT_I2C_QUEUE_DEPTH];
    byte _head  = 0;
    byte _tail  = 0;
    byte _count = 0;

    uint32_t _halfPeriodUs;
    uint32_t _lastPhaseUs = 0;

    byte _byteIndex = 0;
    byte _bitIndex = 0;
    byte _currentByte = 0;
    bool _nakError = false;

    enum class State : byte {
      IDLE,
      START_SDA_LOW,
      START_SCL_LOW,
      BIT_SDA_SET,
      BIT_SCL_HIGH,
      BIT_SCL_LOW,
      ACK_SDA_RELEASE,
      ACK_SCL_HIGH,
      ACK_SCL_LOW,
      STOP_SDA_LOW,
      STOP_SCL_HIGH,
      STOP_SDA_HIGH,
    };

    State _state = State::IDLE;

    void _startJob();
    void _loadNextByte();
    void _finishJob(byte error);
};
