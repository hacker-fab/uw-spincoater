#include "async_lcd.h"


#define LCD_DEBUG 0

#if LCD_DEBUG
  #define LCD_LOG(msg) Serial.println(F(msg))
  #define LCD_LOGF(fmt, ...) { char _b[64]; snprintf(_b, sizeof(_b), fmt, ##__VA_ARGS__); Serial.println(_b); }
#else
  #define LCD_LOG(msg)
  #define LCD_LOGF(fmt, ...)
#endif

AsyncLCD::AsyncLCD(SoftI2C& bus, byte i2cAddr, const byte LCD_COLS, const byte LCD_ROWS): _bus(bus), _addr(i2cAddr), _LCD_COLS(LCD_COLS), _LCD_ROWS(LCD_ROWS) {}

bool AsyncLCD::_enqueueOp(const LcdOp& op) {
  if (_opCount >= LCD_QUEUE_DEPTH) {
    LCD_LOG("[LCD] _enqueueOp REJECTED — LCD queue full");
    return false;
  }
  _ops[_opTail] = op;
  _opTail = (_opTail + 1) % LCD_QUEUE_DEPTH;
  _opCount++;
  return true;
}

bool AsyncLCD::_enqueueDelay(unsigned int ms) {
  LcdOp op{};
  op.type = OpType::DELAY_MS;
  op.delayMs = ms;
  return _enqueueOp(op);
}

void AsyncLCD::_dispatchFront() {
  const LcdOp& op = _ops[_opHead];

  if (op.type == OpType::DELAY_MS) {
    LCD_LOGF("[LCD] delay %u ms (queue=%u)", op.delayMs, _opCount - 1);
    _delayActive = true;
    _delayEndMs = millis() + op.delayMs;
    _opHead = (_opHead + 1) % LCD_QUEUE_DEPTH;
    _opCount--;
    return;
  }

  byte buf[SOFT_I2C_MAX_PAYLOAD];
  byte totalLen = 0;

  if (op.type == OpType::COMMAND) {
    buf[0] = 0x00;
    buf[1] = op.payload[0];
    totalLen = 2;
    LCD_LOGF("[LCD] dispatch CMD  0x%02X (queue=%u)", op.payload[0], _opCount - 1);
  } else {
    buf[0] = 0x40;
    memcpy(buf + 1, op.payload, op.len);
    totalLen = op.len + 1;
    LCD_LOGF("[LCD] dispatch DATA len=%u (queue=%u)", op.len, _opCount - 1);
  }

  if (_bus.write(_addr, buf, totalLen)) {
    _opHead = (_opHead + 1) % LCD_QUEUE_DEPTH;
    _opCount--;
  } else {
    LCD_LOG("[LCD] _dispatchFront: I2C queue full, will retry");
  }
}

void AsyncLCD::begin() {
  enqueueCommand(_LCD_ROWS > 1 ? 0x38 : 0x30); // normal instruction set
  enqueueCommand(_LCD_ROWS > 1 ? 0x39 : 0x31); // extended instruction set
  enqueueCommand(0x14); // 1/5 bias
  enqueueCommand(0x78); // contrast set (low byte)
  enqueueCommand(0x5E); // power/icon/contrast (high byte)
  enqueueCommand(0x6D); // follower control
  enqueueCommand(_LCD_ROWS > 1 ? 0x38 : 0x30); // return to normal instruction set
  enqueueCommand(0x0C); // display on, cursor off, blink off
  enqueueCommand(0x01); _enqueueDelay(2);  // clear display
  enqueueCommand(0x06); // entry mode: increment, no shift
}

void AsyncLCD::tick() {
  if (_delayActive) {
    if ((int32_t)(millis() - _delayEndMs) < 0)
      return;
    _delayActive = false;
  }

  if (_opCount > 0 && _bus.idle()) {
    _dispatchFront();
  }
}

bool AsyncLCD::idle() const {
  return !_delayActive && _opCount == 0 && _bus.idle();
}

bool AsyncLCD::enqueueCommand(byte cmd) {
  LcdOp op{};
  op.type = OpType::COMMAND;
  op.payload[0] = cmd;
  op.len = 1;
  return _enqueueOp(op);
}

bool AsyncLCD::enqueueData(const byte* data, byte len) {
  LcdOp op{};
  op.type = OpType::DATA;
  op.len = min(len, (byte)(SOFT_I2C_MAX_PAYLOAD - 1));
  memcpy(op.payload, data, op.len);
  return _enqueueOp(op);
}

void AsyncLCD::clear() {
  enqueueCommand(0x01);
  _enqueueDelay(2);
}

void AsyncLCD::setCursor(byte col, byte row) {
  static const byte rowOffsets[2] = {0x00, 0x40};
  row = min(row, (byte)(_LCD_ROWS - 1));
  col = min(col, (byte)(_LCD_COLS - 1));
  enqueueCommand(0x80 | (col + rowOffsets[row]));
}

void AsyncLCD::cursor() {
  _disp |= 0x02;
  enqueueCommand(_disp);
}

void AsyncLCD::noCursor() {
  _disp &= 0xFD;
  enqueueCommand(_disp);
}

void AsyncLCD::blink() {
  _disp |= 0x01;
  enqueueCommand(_disp);
}

void AsyncLCD::noBlink() {
  _disp &= 0xFE;
  enqueueCommand(_disp);
}

void AsyncLCD::print(const char* str) {
  if (!str) return;
  byte len = (byte)strlen(str);
  byte maxChunk = SOFT_I2C_MAX_PAYLOAD - 1;

  for (byte offset = 0; offset < len; offset += maxChunk) {
    byte chunk = min(maxChunk, (byte)(len - offset));
    enqueueData((const byte*)str + offset, chunk);
  }
}

void AsyncLCD::print(int val) {
  char buf[6];
  itoa(val, buf, 10);
  print(buf);
}

void AsyncLCD::printSpacePaddedInt(int val) {
  char buf[6];
  sprintf(buf, "%5d", val);
  print(buf);
}

void AsyncLCD::printZeroPaddedInt(int val) {
  char buf[3];
  sprintf(buf, "%02d", val);
  print(buf);
}

void AsyncLCD::write(byte ch) {
  byte b = ch;
  enqueueData(&b, 1);
}

void AsyncLCD::createChar(byte slot, const byte bitmap[8]) {
  enqueueCommand(0x40 | ((slot & 0x07) << 3));

  for (byte row = 0; row < 8; row++) {
    byte b = bitmap[row] & 0x1F;
    enqueueData(&b, 1);
  }
}
