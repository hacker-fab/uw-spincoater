#define ENC_A 7
#define ENC_B 5
#define ENC_C 6
#define INHA 14
#define INLA 11
#define INHB 13
#define INLB 10
#define INHC 12
#define INLC 9

struct encSetup {
  const byte PIN;
  const byte mask;
};

volatile byte encState;
volatile bool commutating;
unsigned int duty;


encSetup a = {ENC_A, 1};
encSetup b = {ENC_B, 2};
encSetup c = {ENC_C, 4};

void ARDUINO_ISR_ATTR encChange(void *arg) {
  encSetup *s = static_cast<encSetup *>(arg);
  if (digitalRead(s->PIN)) {
    encState |= s->mask;
  } else {
    encState &= ~(s->mask);
  }
  commutating = true;
}

void changeMotorState() {
  switch (encState) {
    case 3:
      ledcWrite(INHC, 0);
      digitalWrite(INLC, HIGH);
      ledcWrite(INHA, 0);
      digitalWrite(INLA, LOW);
      ledcWrite(INHB, duty);
      digitalWrite(INLB, LOW);
      break;
    case 1:
      ledcWrite(INHC, 0);
      digitalWrite(INLC, HIGH);
      ledcWrite(INHB, 0);
      digitalWrite(INLB, LOW);
      ledcWrite(INHA, duty);
      digitalWrite(INLA, LOW);
      break;
    case 5:
      ledcWrite(INHB, 0);
      digitalWrite(INLB, HIGH);
      ledcWrite(INHC, 0);
      digitalWrite(INLC, LOW);
      ledcWrite(INHA, duty);
      digitalWrite(INLA, LOW);
      break;
    case 4:
      ledcWrite(INHC, duty);
      digitalWrite(INLC, LOW);
      ledcWrite(INHB, 0);
      digitalWrite(INLB, HIGH);
      ledcWrite(INHA, 0);
      digitalWrite(INLA, LOW);
      break;
    case 6:
      ledcWrite(INHA, 0);
      digitalWrite(INLA, HIGH);
      ledcWrite(INHB, 0);
      digitalWrite(INLB, LOW);
      ledcWrite(INHC, duty);
      digitalWrite(INLC, LOW);
      break;
    case 2:
      ledcWrite(INHA, 0);
      digitalWrite(INLA, HIGH);
      ledcWrite(INHC, 0);
      digitalWrite(INLC, LOW);
      ledcWrite(INHB, duty);
      digitalWrite(INLB, LOW);
      break;
    default:
      ledcWrite(INHA, 0);
      digitalWrite(INLA, LOW);
      ledcWrite(INHB, 0);
      digitalWrite(INLB, LOW);
      ledcWrite(INHC, 0);
      digitalWrite(INLC, LOW);
      break;
  }
}

void setup() {
  Serial.begin(9600);
  pinMode(ENC_A, INPUT);
  pinMode(ENC_B, INPUT);
  pinMode(ENC_C, INPUT);
  pinMode(INHA, OUTPUT);
  pinMode(INLA, OUTPUT);
  pinMode(INHB, OUTPUT);
  pinMode(INLB, OUTPUT);
  pinMode(INHC, OUTPUT);
  pinMode(INLC, OUTPUT);
  encState |= digitalRead(ENC_A);
  encState |= (digitalRead(ENC_B) << 1);
  encState |= (digitalRead(ENC_C) << 2);
  attachInterruptArg(digitalPinToInterrupt(a.PIN), encChange, &a, CHANGE);
  attachInterruptArg(digitalPinToInterrupt(b.PIN), encChange, &b, CHANGE);
  attachInterruptArg(digitalPinToInterrupt(c.PIN), encChange, &c, CHANGE);
  ledcSetClockSource(LEDC_USE_APB_CLK);
  ledcWrite(INHA, 0);
  ledcWrite(INHB, 0);
  ledcWrite(INHC, 0);
  ledcAttach(INHA, 16000, 12);
  ledcAttach(INHB, 16000, 12);
  ledcAttach(INHC, 16000, 12);
  duty = 100;
  changeMotorState();
  delay(1000);
}

void loop() {
  if (duty < 1000) {
    duty += 2;
  }
  if (commutating) {
    changeMotorState();
    commutating = false;
  }
  delay(10);
}
