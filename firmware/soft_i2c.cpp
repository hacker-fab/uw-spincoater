#include "soft_i2c.h"


#define SOFT_I2C_DEBUG 0

#if SOFT_I2C_DEBUG
  #define I2C_LOG(msg) Serial.println(F(msg))
  #define I2C_LOGF(fmt, ...) { char _b[64]; snprintf(_b, sizeof(_b), fmt, ##__VA_ARGS__); Serial.println(_b); }
#else
  #define I2C_LOG(msg)
  #define I2C_LOGF(fmt, ...)
#endif

SoftI2C::SoftI2C(byte sdaPin, byte sclPin, unsigned long freqHz) : _sda(sdaPin), _scl(sclPin) {
  _halfPeriodUs = 500000UL / freqHz;
}

void SoftI2C::begin() {
  sdaHigh();
  sclHigh();
  _lastPhaseUs = micros();
}

bool SoftI2C::write(byte addr, const byte* data, byte len, I2CCallback cb) {
  if (_count >= SOFT_I2C_QUEUE_DEPTH || len > SOFT_I2C_MAX_PAYLOAD) {
    I2C_LOGF("[I2C] write() REJECTED addr=0x%02X len=%u count=%u", addr, len, _count);
    return false;
  }

  I2CJob& job = _queue[_tail];
  job.addr = addr;
  job.len = len;
  job.cb = cb;
  memcpy(job.data, data, len);

  _tail = (_tail + 1) % SOFT_I2C_QUEUE_DEPTH;
  _count++;
  return true;
}

void SoftI2C::_startJob() {
  _byteIndex = 1;
  _bitIndex = 7;
  _nakError = false;
  _currentByte = (_queue[_head].addr << 1) & 0xFE;
  _state = State::START_SDA_LOW;
  I2C_LOGF("[I2C] job start  addr=0x%02X len=%u queue=%u", _queue[_head].addr, _queue[_head].len, _count);
}

void SoftI2C::_loadNextByte() {
  byte dataIndex = _byteIndex - 1;
  _byteIndex++;
  if (dataIndex < _queue[_head].len) {
    _currentByte = _queue[_head].data[dataIndex];
    _bitIndex = 7;
    _state = State::BIT_SDA_SET;
  } else {
    _state = State::STOP_SDA_LOW;
  }
}

void SoftI2C::_finishJob(byte error) {
  I2C_LOGF("[I2C] job finish addr=0x%02X err=%u queue=%u", _queue[_head].addr, error, _count - 1);
  I2CCallback cb = _queue[_head].cb;
  _head = (_head + 1) % SOFT_I2C_QUEUE_DEPTH;
  _count--;
  _state = State::IDLE;
  if (cb) cb(error);
}

bool SoftI2C::tick() {
  if (_state == State::IDLE) {
    if (_count == 0) return false;
    _startJob();
  }

  if ((micros() - _lastPhaseUs) < _halfPeriodUs)
    return false;

  switch (_state) {
    case State::START_SDA_LOW:
      sdaLow();
      _state = State::START_SCL_LOW;
      break;

    case State::START_SCL_LOW:
      sclLow();
      _state = State::BIT_SDA_SET;
      break;

    case State::BIT_SDA_SET:
      if (_currentByte & (1 << _bitIndex)) {
        sdaHigh();
      }
      else {
        sdaLow();
      }
      _state = State::BIT_SCL_HIGH;
      break;

    case State::BIT_SCL_HIGH:
      sclHigh();
      if (!sclRead()) return true;
      _state = State::BIT_SCL_LOW;
      break;

    case State::BIT_SCL_LOW:
      sclLow();
      if (_bitIndex > 0) {
        _bitIndex--;
        _state = State::BIT_SDA_SET;
      } else {
        _state = State::ACK_SDA_RELEASE;
      }
      break;

    case State::ACK_SDA_RELEASE:
      sdaHigh();
      _state = State::ACK_SCL_HIGH;
      break;

    case State::ACK_SCL_HIGH:
      sclHigh();
      if (!sclRead()) return true;
      _state = State::ACK_SCL_LOW;
      break;

    case State::ACK_SCL_LOW:
      if (sdaRead()) {
        _nakError = true;
        I2C_LOGF("[I2C] NAK  byte=%u addr=0x%02X", _byteIndex, _queue[_head].addr);
      } else {
        I2C_LOGF("[I2C] ACK  byte=%u", _byteIndex);
      }
      sclLow();
      if (_nakError) {
        _state = State::STOP_SDA_LOW;
      }
      else {
        _loadNextByte();
      }
      break;

    case State::STOP_SDA_LOW:
      sdaLow();
      _state = State::STOP_SCL_HIGH;
      break;

    case State::STOP_SCL_HIGH:
      sclHigh();
      _state = State::STOP_SDA_HIGH;
      break;

    case State::STOP_SDA_HIGH:
      sdaHigh();
      _finishJob(_nakError ? 1 : 0);
      break;

    default: break;
  }

  _lastPhaseUs = micros();
  return true;
}
