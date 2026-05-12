#include <SimpleFOC.h>
#include <Keypad.h>
#include "soft_i2c.h"
#include "async_lcd.h"

#define SDA_PIN 1
#define SCL_PIN 2
#define ENC_A 7
#define ENC_B 5
#define ENC_C 6
#define INHA 14
#define INLA 11
#define INHB 13
#define INLB 10
#define INHC 12
#define INLC 9
#define ROW1 47
#define ROW2 38
#define ROW3 37
#define ROW4 35
#define COL1 48
#define COL2 21
#define COL3 36
#define START_BTN 8

const byte KEYPAD_ROWS = 4;
const byte KEYPAD_COLS = 3;
char keys[KEYPAD_ROWS][KEYPAD_COLS] = {
  {'1','2','3'},
  {'4','5','6'},
  {'7','8','9'},
  {'*','0','#'}
};
byte rowPins[KEYPAD_ROWS] = {ROW1, ROW2, ROW3, ROW4};
byte colPins[KEYPAD_COLS] = {COL1, COL2, COL3};
Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, KEYPAD_ROWS, KEYPAD_COLS);

BLDCMotor motor = BLDCMotor(1, 0.112, 2000);  // Assuming delta wiring, tuned to 2000 KV
BLDCDriver6PWM driver = BLDCDriver6PWM(INHA, INLA, INHB, INLB, INHC, INLC);
HallSensor sensor = HallSensor(ENC_A, ENC_B, ENC_C, 1);

SoftI2C  I2CBus(SDA_PIN, SCL_PIN, 100000);
AsyncLCD LCD(I2CBus, 0x3C, 20, 2);

float targetFix = 1.02;
float accelCap = 100; // rads/s^2

volatile float targetVel;
volatile float limitedVel;

volatile int encCount;

unsigned long currTime;
unsigned long tloop;
unsigned long lastTime;
unsigned long lastLCDTick;
unsigned long lastSpeedUpdateTime;
unsigned long lastBtnTime;
unsigned long stoppedBlinkTimer;

bool inputState;
bool stopState;
bool stopped = true;
bool pressed;
int setSpeed;
int tempSetSpeed;

byte plusMinus[] = {0x04, 0x04, 0x1F, 0x04, 0x04, 0x00, 0x1F, 0x00};

void doA(){encCount++; sensor.handleA();}
void doB(){encCount++; sensor.handleB();}
void doC(){encCount++; sensor.handleC();}

void setMotorSpeed(float vel) {
  float outVel = vel; // Another variable in case interrupt happens while constraining
  if (vel < 0) {
    outVel *= -1;
  }
  if (vel < 20 && vel != 0) {
    outVel = 20;
  } else if (vel > 1676) {
    outVel = 1676;
  }
  targetVel = outVel * targetFix;
}

void setup() {
  driver.pwm_frequency = 20000;
  driver.voltage_power_supply = 12;
  driver.init();
  
  sensor.velocity_max = 2000;
  
  sensor.enableInterrupts(doA, doB, doC);
  sensor.init();

  motor.foc_modulation = FOCModulationType::Trapezoid_120;
  motor.torque_controller = TorqueControlType::estimated_current;
  motor.controller = MotionControlType::velocity;
  motor.updateVoltageLimit(12);
  motor.updateCurrentLimit(30); // Arbitrary, does not actually match current

  motor.linkDriver(&driver);
  motor.linkSensor(&sensor);

  motor.zero_electric_angle = 1.047;
  motor.sensor_direction = Direction::CW;

  motor.PID_velocity.P = 0.05;
  motor.PID_velocity.I = 0.4;
  motor.PID_velocity.D = 0;
  motor.LPF_velocity.Tf = 0.0001;
  
  motor.init();
  motor.initFOC();

  I2CBus.begin();
  LCD.begin();
  LCD.createChar(0, plusMinus);
  LCD.setCursor(0, 0);
  LCD.print("Actual:     0");
  LCD.write(0);
  LCD.print("5 RPM");
  LCD.setCursor(0, 1);
  LCD.print("      Stopped");

  pinMode(START_BTN, INPUT);
}

// Max target: 1676 rad/s (~16k RPM), Min target: 20 rad/s (~200 RPM)

void loop() {
  currTime = micros();
  tloop = micros() - lastTime;
  lastTime = currTime;
  float step = accelCap * tloop / 1000000;
  if (limitedVel <= targetVel - step) {
    limitedVel += step;
  } else if (limitedVel >= targetVel + step) {
    limitedVel -= step;
  } else {
    limitedVel = targetVel;
  }
  motor.loopFOC();
  motor.move(limitedVel);

  if (currTime - lastLCDTick >= 100) {
    I2CBus.tick();
    LCD.tick();
    lastLCDTick = micros();
  }
  
  if (currTime - lastBtnTime >= 1000 && !inputState) {
    if (digitalRead(START_BTN)) {
      if (pressed) {
        stopped = !stopped;
        stoppedBlinkTimer = micros();
        stopState = stopped;
        if (stopped) {
          setMotorSpeed(0);
        } else {
          setMotorSpeed(setSpeed * 6.2832 / 60);
        }
        pressed = false;
      }
    } else {
      if (!pressed) {
        pressed = true;
      }
    }
    lastBtnTime = micros();
  }
  
  if (currTime - stoppedBlinkTimer >= 1000000 && !inputState) {
    LCD.setCursor(0, 1);
    if (!stopState) {
      LCD.print(" Set: ");
      LCD.printSpacePaddedInt(setSpeed);
      LCD.write(0);
      LCD.print("1% RPM");
      if (stopped) {
        stopState = true;
      }
    } else if (stopped) {
      LCD.print("      Stopped       ");
      stopState = false;
    }
    stoppedBlinkTimer = micros();
  }
  
  char key = keypad.getKey(); // does not taste better than bread
  if (key) {
      if (key != '*' && key != '#' && !inputState) {
        tempSetSpeed = 0;
        LCD.setCursor(0, 1);
        LCD.print(" Set:     0");
        LCD.write(0);
        LCD.print("1% RPM");
        LCD.setCursor(10, 1);
        LCD.cursor();
        LCD.blink();
        inputState = true;
      }
      if (inputState) {
        if (key == '*') {
          if (tempSetSpeed > 0) {
            tempSetSpeed /= 10;
            LCD.setCursor(6, 1);
            LCD.printSpacePaddedInt(tempSetSpeed);
            LCD.setCursor(10, 1);
          } else {
            LCD.noCursor();
            LCD.noBlink();
            inputState = false;
          }
        } else if (key == '#') {
          if (tempSetSpeed > 16000) {
            tempSetSpeed = 16000;
          }
          if (tempSetSpeed < 200 && tempSetSpeed != 0) {
            tempSetSpeed = 200;
          }
          setSpeed = tempSetSpeed;
          if (!stopped) {
            setMotorSpeed(setSpeed * 6.2832 / 60);
          }
          LCD.noCursor();
          LCD.noBlink();
          inputState = false;
        } else if (tempSetSpeed < 10000) {
          tempSetSpeed = tempSetSpeed * 10 + key - 0x30;
          LCD.setCursor(6, 1);
          LCD.printSpacePaddedInt(tempSetSpeed);
          LCD.setCursor(10, 1);
        }
      }
  }
  
  if (currTime - lastSpeedUpdateTime >= 2000000) {
    LCD.setCursor(8, 0);
    LCD.printSpacePaddedInt(encCount * 5);
    if (inputState) {
      LCD.setCursor(10, 1);
    }
    encCount = 0;
    lastSpeedUpdateTime = micros();
  }
}
