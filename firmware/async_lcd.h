#pragma once
#include "soft_i2c.h"

// ─────────────────────────────────────────────────────────────────────────────
// Usage:
//   SoftI2C  myBus(SDA_PIN, SCL_PIN, 100000UL);
//   AsyncLCD myLCD(myBus, 0x3C);
//
//   void setup() {
//       myBus.begin();
//       myLCD.begin();
//   }
//
//   void loop() {
//       myBus.tick();
//       myLCD.tick();
//   }
//
//   myLCD.clear();
//   myLCD.setCursor(0, 0);
//   myLCD.print("Hello");
//   myLCD.print(42);
//
//   byte heart[8] = { 0x00, 0x0A, 0x1F, 0x1F, 0x0E, 0x04, 0x00, 0x00 };
//   myLCD.createChar(0, heart);
//   myLCD.setCursor(0, 0);
//   myLCD.write(0);
//
// ─────────────────────────────────────────────────────────────────────────────

#define LCD_QUEUE_DEPTH 32

class AsyncLCD {
  public:
    AsyncLCD(SoftI2C& bus, byte i2cAddr, byte LCD_COLS, byte LCD_ROWS);

    void begin();

    void tick();

    bool idle() const;

    void clear();
    void setCursor(byte col, byte row);
    void print(const char* str);
    void print(int val);
    void printSpacePaddedInt(int val);
    void write(byte ch);
    void createChar(byte slot, const byte bitmap[8]);
    void cursor();
    void noCursor();
    void blink();
    void noBlink();

    bool enqueueCommand(byte cmd);
    bool enqueueData(const byte* data, byte len);

  private:
    SoftI2C& _bus;
    byte _addr;
    const byte _LCD_COLS;
    const byte _LCD_ROWS;
    byte _disp = 0x0C;

    enum class OpType : byte { COMMAND, DATA, DELAY_MS };

    struct LcdOp {
      OpType type;
      byte payload[SOFT_I2C_MAX_PAYLOAD - 1];
      byte len;
      unsigned int delayMs;
    };

    LcdOp _ops[LCD_QUEUE_DEPTH];
    byte _opHead = 0;
    byte _opTail = 0;
    byte _opCount = 0;

    bool _delayActive = false;
    unsigned long _delayEndMs = 0;

    bool _enqueueOp(const LcdOp& op);
    bool _enqueueDelay(unsigned int ms);
    void _dispatchFront();
};
