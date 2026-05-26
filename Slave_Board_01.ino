#include "HX711.h"

// =====================================================
// HX711 PINS
// =====================================================
#define HX1_DT   8
#define HX1_SCK  9

#define HX2_DT   6
#define HX2_SCK  7

#define HX3_DT   4
#define HX3_SCK  5

#define HX4_DT   10
#define HX4_SCK  11

// =====================================================
// LED PINS
// =====================================================
#define LED1 12
#define LED2 13
#define LED3 14
#define LED4 15

// =====================================================
// MOTOR 1
// =====================================================
#define M1_STEP 16
#define M1_DIR  17
#define M1_EN   18

// =====================================================
// MOTOR 2
// =====================================================
#define M2_STEP 19
#define M2_DIR  20
#define M2_EN   21

// =====================================================
// LIMIT SWITCHES
// =====================================================
#define U1_TOWARD_LIMIT 1
#define U1_AWAY_LIMIT   2

#define U2_TOWARD_LIMIT 42
#define U2_AWAY_LIMIT   41

// =====================================================
// MOTOR SETTINGS
// =====================================================
#define DIR_TOWARD HIGH
#define DIR_AWAY   LOW

#define STEP_DELAY_US 60
#define STEP_PULSE_US 5

// =====================================================
// HX711 OBJECTS
// =====================================================
HX711 scale1;
HX711 scale2;
HX711 scale3;
HX711 scale4;

// =====================================================
// CALIBRATION
// =====================================================
float cal1 = 639.21;
float cal2 = 635.30;
float cal3 = 654.45;
float cal4 = 635.85;

// =====================================================
// LOAD CELL STATES
// =====================================================
bool lc1Active = true;
bool lc2Active = true;
bool lc3Active = true;
bool lc4Active = true;

bool lc1Removed = false;
bool lc2Removed = false;
bool lc3Removed = false;
bool lc4Removed = false;

// =====================================================
// HX FAIL COUNTERS
// =====================================================
int fail1 = 0;
int fail2 = 0;
int fail3 = 0;
int fail4 = 0;

const int FAIL_LIMIT = 2;

// =====================================================
// STUCK DETECTION
// =====================================================
float lastW1 = 0;
float lastW2 = 0;
float lastW3 = 0;
float lastW4 = 0;

int stuck1 = 0;
int stuck2 = 0;
int stuck3 = 0;
int stuck4 = 0;

const int STUCK_LIMIT = 10;

// =====================================================
// WEIGHTS
// =====================================================
float w1 = 0;
float w2 = 0;
float w3 = 0;
float w4 = 0;

float totalWeight = 0;
float teaWeight = 0;

// =====================================================
// HOPPER
// =====================================================
bool hopperPlaced = false;
bool hopperRecorded = false;

float hopperWeight = 0;

// =====================================================
// MOTOR POSITION MEMORY
// =====================================================
long posM1 = 0;
long posM2 = 0;

// =====================================================
// LED CONTROL
// =====================================================
void allLEDsOff()
{
  digitalWrite(LED1, LOW);
  digitalWrite(LED2, LOW);
  digitalWrite(LED3, LOW);
  digitalWrite(LED4, LOW);
}

void showFaultLED(int cell)
{
  allLEDsOff();

  if(cell == 1) digitalWrite(LED1, HIGH);
  if(cell == 2) digitalWrite(LED2, HIGH);
  if(cell == 3) digitalWrite(LED3, HIGH);
  if(cell == 4) digitalWrite(LED4, HIGH);
}

// =====================================================
// MOTOR CONTROL
// =====================================================
void enableMotor(int en)
{
  digitalWrite(en, LOW);
}

void disableMotor(int en)
{
  digitalWrite(en, HIGH);
}

void pulseStep(int stepPin)
{
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(STEP_PULSE_US);

  digitalWrite(stepPin, LOW);
  delayMicroseconds(STEP_DELAY_US);
}

void stepMotor(int stepPin, bool toward, long &pos)
{
  pulseStep(stepPin);

  if(toward) pos++;
  else pos--;
}

// =====================================================
// MOVE MOTOR UNTIL LIMIT
// =====================================================
bool moveUntilLimit(
  int en,
  int dir,
  int step,
  bool toward,
  int limitPin,
  long &pos)
{
  enableMotor(en);

  digitalWrite(dir, toward ? DIR_TOWARD : DIR_AWAY);

  while(digitalRead(limitPin) != LOW)
  {
    stepMotor(step, toward, pos);
  }

  disableMotor(en);

  return true;
}

// =====================================================
// REMOVE LOAD CELL
// =====================================================
bool removeLoadCell(int cell)
{
  Serial.print("REMOVING LOAD CELL ");
  Serial.println(cell);

  switch(cell)
  {
    case 1:
      return moveUntilLimit(
        M1_EN,
        M1_DIR,
        M1_STEP,
        true,
        U1_TOWARD_LIMIT,
        posM1);

    case 2:
      return moveUntilLimit(
        M1_EN,
        M1_DIR,
        M1_STEP,
        false,
        U1_AWAY_LIMIT,
        posM1);

    case 3:
      return moveUntilLimit(
        M2_EN,
        M2_DIR,
        M2_STEP,
        true,
        U2_TOWARD_LIMIT,
        posM2);

    case 4:
      return moveUntilLimit(
        M2_EN,
        M2_DIR,
        M2_STEP,
        false,
        U2_AWAY_LIMIT,
        posM2);
  }

  return false;
}

// =====================================================
// HOME SYSTEM
// =====================================================
void homeSystem()
{
  Serial.println("HOME START");

  enableMotor(M1_EN);
  enableMotor(M2_EN);

  while(posM1 != 0)
  {
    bool toward = (posM1 < 0);

    digitalWrite(
      M1_DIR,
      toward ? DIR_TOWARD : DIR_AWAY);

    stepMotor(M1_STEP, toward, posM1);
  }

  while(posM2 != 0)
  {
    bool toward = (posM2 < 0);

    digitalWrite(
      M2_DIR,
      toward ? DIR_TOWARD : DIR_AWAY);

    stepMotor(M2_STEP, toward, posM2);
  }

  disableMotor(M1_EN);
  disableMotor(M2_EN);

  lc1Active = true;
  lc2Active = true;
  lc3Active = true;
  lc4Active = true;

  lc1Removed = false;
  lc2Removed = false;
  lc3Removed = false;
  lc4Removed = false;

  fail1 = fail2 = fail3 = fail4 = 0;

  stuck1 = stuck2 = stuck3 = stuck4 = 0;

  allLEDsOff();

  Serial.println("HOME COMPLETE");
}

// =====================================================
// HANDLE FAULT
// =====================================================
void handleFault(int cell)
{
  Serial.print("FAULT DETECTED ON CELL ");
  Serial.println(cell);

  showFaultLED(cell);

  if(cell == 1)
  {
    if(lc1Removed) return;
    lc1Active = false;
    lc1Removed = true;
  }

  if(cell == 2)
  {
    if(lc2Removed) return;
    lc2Active = false;
    lc2Removed = true;
  }

  if(cell == 3)
  {
    if(lc3Removed) return;
    lc3Active = false;
    lc3Removed = true;
  }

  if(cell == 4)
  {
    if(lc4Removed) return;
    lc4Active = false;
    lc4Removed = true;
  }

  removeLoadCell(cell);

  Serial.println("SYSTEM REBALANCED");
}

// =====================================================
// SAFE READ
// =====================================================
float safeRead(
  HX711 &scale,
  int cell,
  int &failCount,
  int &stuckCount,
  float &lastValue,
  bool active)
{
  if(!active) return 0;

  // =========================================
  // HX READY CHECK
  // =========================================
  if(!scale.is_ready())
  {
    failCount++;

    Serial.print("HX");
    Serial.print(cell);
    Serial.println(" NOT READY");

    if(failCount >= FAIL_LIMIT)
    {
      handleFault(cell);
    }

    return lastValue;
  }

  // =========================================
  // GET VALUE
  // =========================================
  float val = scale.get_units(1);

  // =========================================
  // INVALID VALUE
  // =========================================
  if(isnan(val) || abs(val) > 100000)
  {
    failCount++;

    Serial.print("HX");
    Serial.print(cell);
    Serial.println(" INVALID");

    if(failCount >= FAIL_LIMIT)
    {
      handleFault(cell);
    }

    return lastValue;
  }

  failCount = 0;

  // =========================================
  // STUCK DETECTION
  // =========================================
  if(abs(val - lastValue) < 0.01)
  {
    stuckCount++;
  }
  else
  {
    stuckCount = 0;
  }

  // =========================================
  // STUCK / DISCONNECTED
  // =========================================
  if(stuckCount >= STUCK_LIMIT)
  {
    Serial.print("HX");
    Serial.print(cell);
    Serial.println(" STUCK / DISCONNECTED");

    handleFault(cell);
  }

  lastValue = val;

  return val;
}

// =====================================================
// READ ALL CELLS
// =====================================================
void readCells()
{
  w1 = safeRead(
    scale1,
    1,
    fail1,
    stuck1,
    lastW1,
    lc1Active);

  w2 = safeRead(
    scale2,
    2,
    fail2,
    stuck2,
    lastW2,
    lc2Active);

  w3 = safeRead(
    scale3,
    3,
    fail3,
    stuck3,
    lastW3,
    lc3Active);

  w4 = safeRead(
    scale4,
    4,
    fail4,
    stuck4,
    lastW4,
    lc4Active);

  totalWeight = 0;

  if(lc1Active) totalWeight += w1;
  if(lc2Active) totalWeight += w2;
  if(lc3Active) totalWeight += w3;
  if(lc4Active) totalWeight += w4;

  // =========================================
  // TEA WEIGHT
  // =========================================
  if(hopperRecorded)
  {
    teaWeight = totalWeight - hopperWeight;

    if(teaWeight < 0)
      teaWeight = 0;
  }
  else
  {
    teaWeight = totalWeight;
  }

  Serial.print("W1:");
  Serial.print(w1);

  Serial.print(" W2:");
  Serial.print(w2);

  Serial.print(" W3:");
  Serial.print(w3);

  Serial.print(" W4:");
  Serial.print(w4);

  Serial.print(" TOTAL:");
  Serial.print(totalWeight);

  Serial.print(" TEA:");
  Serial.println(teaWeight);
}

// =====================================================
// SERIAL COMMANDS
// =====================================================
void handleSerial()
{
  if(!Serial.available()) return;

  String cmd = Serial.readStringUntil('\n');
  cmd.trim();

  // =========================================
  // K = HOPPER PLACED
  // =========================================
  if(cmd == "K")
  {
    hopperPlaced = true;

    Serial.println("HOPPER PLACED");
  }

  // =========================================
  // W = RECORD HOPPER WEIGHT
  // =========================================
  else if(cmd == "W")
  {
    if(!hopperPlaced)
    {
      Serial.println("PLACE HOPPER FIRST");
      return;
    }

    hopperWeight = totalWeight;

    hopperRecorded = true;

    Serial.print("HOPPER WEIGHT RECORDED: ");
    Serial.println(hopperWeight);
  }

  // =========================================
  // H = HOME
  // =========================================
  else if(cmd == "H")
  {
    homeSystem();
  }

  // =========================================
  // R = RESET
  // =========================================
  else if(cmd == "R")
  {
    Serial.println("RESET");

    hopperPlaced = false;
    hopperRecorded = false;

    hopperWeight = 0;

    homeSystem();
  }
}

// =====================================================
// SETUP
// =====================================================
void setup()
{
  Serial.begin(115200);

  // =========================================
  // LEDS
  // =========================================
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  pinMode(LED4, OUTPUT);

  // =========================================
  // MOTORS
  // =========================================
  pinMode(M1_STEP, OUTPUT);
  pinMode(M1_DIR, OUTPUT);
  pinMode(M1_EN, OUTPUT);

  pinMode(M2_STEP, OUTPUT);
  pinMode(M2_DIR, OUTPUT);
  pinMode(M2_EN, OUTPUT);

  // =========================================
  // LIMIT SWITCHES
  // =========================================
  pinMode(U1_TOWARD_LIMIT, INPUT_PULLUP);
  pinMode(U1_AWAY_LIMIT, INPUT_PULLUP);

  pinMode(U2_TOWARD_LIMIT, INPUT_PULLUP);
  pinMode(U2_AWAY_LIMIT, INPUT_PULLUP);

  disableMotor(M1_EN);
  disableMotor(M2_EN);

  // =========================================
  // HX711 START
  // =========================================
  scale1.begin(HX1_DT, HX1_SCK);
  scale2.begin(HX2_DT, HX2_SCK);
  scale3.begin(HX3_DT, HX3_SCK);
  scale4.begin(HX4_DT, HX4_SCK);

  scale1.set_scale(cal1);
  scale2.set_scale(cal2);
  scale3.set_scale(cal3);
  scale4.set_scale(cal4);

  Serial.println("SYSTEM READY");
  Serial.println("K = Hopper placed");
  Serial.println("W = Record hopper weight");
  Serial.println("H = Home system");
  Serial.println("R = Reset system");
}

// =====================================================
// LOOP
// =====================================================
void loop()
{
  handleSerial();

  readCells();

  delay(300);
}