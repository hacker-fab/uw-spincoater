#include <SimpleFOC.h>
#include <Keypad.h>
#include <Wire.h>
#include <Adafruit_HUSB238.h>
#include "soft_i2c.h"
#include "async_lcd.h"

#define SDA_PIN 15
#define SCL_PIN 16
#define ENC_A 48
#define ENC_B 21
#define ENC_C 47
#define INHA 14
#define INLA 11
#define INHB 13
#define INLB 10
#define INHC 12
#define INLC 9
#define ROW1 41
#define ROW2 36
#define ROW3 37
#define ROW4 39
#define COL1 40
#define COL2 42
#define COL3 38
#define START_BTN 1
#define START_LED 2

Adafruit_HUSB238 husb238;

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

BLDCMotor motor = BLDCMotor(1, 0.15, 1700);  // Assuming delta wiring, tuned to 1700 KV
BLDCDriver6PWM driver = BLDCDriver6PWM(INHA, INLA, INHB, INLB, INHC, INLC);
HallSensor sensor = HallSensor(ENC_A, ENC_B, ENC_C, 1);

SoftI2C  I2CBus(SDA_PIN, SCL_PIN, 100000);
AsyncLCD LCD(I2CBus, 0x3C, 20, 2);

bool powerCheck;


// To Calibrate:
// 1. Set targetFix to 1 and change "currTime - lastSpeedUpdateTime >= 2000000" to "currTime - lastSpeedUpdateTime >= 10000000" and change "encCount * 5" to "encCount"
// 2. Set motor to run at 1k RPM and let run for 20 seconds.
// 3. Save actual speed and targeted speed and increase speed by 1k until 14k RPM.
// 4. Perform linear regression to find targetFix, and undo code edits.
float targetFix = 1.00173; // Magic number idk where this comes from
float accelCap = 200; // rads/s^2

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
  } else if (vel > 1467) {
    outVel = 1467;
  }
  targetVel = outVel * targetFix;
}

void setup() {
  delay(1000);
  Wire.begin(SDA_PIN, SCL_PIN);
  husb238.begin(HUSB238_I2CADDR_DEFAULT, &Wire);
  for (int i = PD_SRC_12V; i >= PD_SRC_5V; i--) {
    if (husb238.isVoltageDetected((HUSB238_PDSelection)i)) {                                                                                                                                                                                                         
      HUSB238_CurrentSetting currentDetected = husb238.currentDetected((HUSB238_PDSelection)i);
      switch ((HUSB238_PDSelection)i) {
        case PD_SRC_5V:
          if (currentDetected >= CURRENT_3_0_A) {
            husb238.selectPD(PD_SRC_5V);
            powerCheck = true;
          }
          break;
        case PD_SRC_9V:
          if (currentDetected >= CURRENT_1_75_A) {
            husb238.selectPD(PD_SRC_9V);
            powerCheck = true;
          }
          break;
        case PD_SRC_12V:
          if (currentDetected >= CURRENT_1_25_A) {
            husb238.selectPD(PD_SRC_12V);
            powerCheck = true;
          }
          break;
        default:
          continue;
      }
      if (powerCheck) {
        husb238.requestPD();
        break;
      }
    }
  }
  Wire.end();
  
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
  motor.updateCurrentLimit(5); // Arbitrary, does not actually match current

  motor.linkDriver(&driver);
  motor.linkSensor(&sensor);

  motor.zero_electric_angle = 1.0472;
  motor.sensor_direction = Direction::CW;

  motor.PID_velocity.P = 0.05;
  motor.PID_velocity.I = 0.3;
  motor.PID_velocity.D = 0;
  motor.LPF_velocity.Tf = 0.0001;
  
  motor.init();
  motor.initFOC();
  
  I2CBus.begin();
  LCD.begin();
  if (powerCheck) {
    LCD.createChar(0, plusMinus);
    LCD.setCursor(0, 0);
    LCD.print("Actual:     0");
    LCD.write(0);
    LCD.print("5 RPM");
    LCD.setCursor(3, 1);
    LCD.print("   Stopped");
  
    pinMode(START_BTN, INPUT);
    pinMode(START_LED, OUTPUT);
  } else {
    LCD.setCursor(1, 0);
    LCD.print("Insufficient Power");
    LCD.setCursor(0, 1);
    LCD.print("Try Different Source");
    for (;;) {
      I2CBus.tick();
      LCD.tick();
      delayMicroseconds(100);
    }
  }
}

// Max target: 1467 rad/s (~14k RPM), Min target: 20 rad/s (~200 RPM)

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
          digitalWrite(START_LED, LOW);
          setMotorSpeed(0);
        } else {
          digitalWrite(START_LED, HIGH);
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
    LCD.setCursor(3, 1);
    if (!stopState) {
      LCD.print("Set: ");
      LCD.printSpacePaddedInt(setSpeed);
      LCD.print(" RPM");
      if (stopped) {
        stopState = true;
      }
    } else if (stopped) {
      LCD.print("   Stopped       ");
      stopState = false;
    }
    stoppedBlinkTimer = micros();
  }
  
  char key = keypad.getKey(); // does not taste better than bread
  if (key) {
      if (key != '*' && key != '#' && !inputState) {
        tempSetSpeed = 0;
        LCD.setCursor(3, 1);
        LCD.print("Set:     0");
        LCD.print(" RPM");
        LCD.setCursor(12, 1);
        LCD.cursor();
        LCD.blink();
        inputState = true;
      }
      if (inputState) {
        if (key == '*') {
          if (tempSetSpeed > 0) {
            tempSetSpeed /= 10;
            LCD.setCursor(8, 1);
            LCD.printSpacePaddedInt(tempSetSpeed);
            LCD.setCursor(12, 1);
          } else {
            LCD.noCursor();
            LCD.noBlink();
            inputState = false;
          }
        } else if (key == '#') {
          if (tempSetSpeed > 14000) {
            tempSetSpeed = 14000;
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
          LCD.setCursor(8, 1);
          LCD.printSpacePaddedInt(tempSetSpeed);
          LCD.setCursor(12, 1);
        }
      }
  }
  
  if (currTime - lastSpeedUpdateTime >= 2000000) {
    LCD.setCursor(8, 0);
    LCD.printSpacePaddedInt(encCount * 5);
    if (inputState) {
      LCD.setCursor(12, 1);
    }
    encCount = 0;
    lastSpeedUpdateTime = micros();
  }
}
