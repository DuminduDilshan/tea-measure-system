// =====================================================
// SLAVE BOARD 01
// Arduino Mega (or compatible)
//
// - 4x HX711 load cells
// - 2x NEMA stepper motors (step/dir/en)
// - 6x limit switches
// - Calculates total weight from active cells (3 or 4)
// - Outputs ONLY clean total weight over Serial
//   Format: "TOTAL:49.43\n"
//   Slave Board 02 reads this line via UART RX
//
// SERIAL COMMANDS (USB Serial Monitor):
//   K   Hopper placed
//   W   Record hopper weight (tare hopper from total)
//   H   Home system (return all cells to active)
//   R   Full reset
// =====================================================

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

float totalWeight = 0;  // sum of active cells
float teaWeight   = 0;  // totalWeight minus hopper

// =====================================================
// HOPPER
// =====================================================
bool  hopperPlaced   = false;
bool  hopperRecorded = false;
float hopperWeight   = 0;

// =====================================================
// MOTOR POSITION MEMORY
// =====================================================
long posM1 = 0;
long posM2 = 0;

// =====================================================
// SERIAL OUTPUT TIMING
// Send total weight at this interval (ms)
// =====================================================
unsigned long lastSendTime = 0;
#define SEND_INTERVAL_MS 300

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
void enableMotor(int en)  { digitalWrite(en, LOW);  }
void disableMotor(int en) { digitalWrite(en, HIGH); }

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
  else       pos--;
}

// =====================================================
// MOVE MOTOR UNTIL LIMIT
// =====================================================
bool moveUntilLimit(
  int en, int dir, int step,
  bool toward, int limitPin, long &pos)
{
  enableMotor(en);
  digitalWrite(dir, toward ? DIR_TOWARD : DIR_AWAY);

  while(digitalRead(limitPin) != LOW)
    stepMotor(step, toward, pos);

  disableMotor(en);
  return true;
}

// =====================================================
// REMOVE LOAD CELL (physical retraction via motor)
// =====================================================
bool removeLoadCell(int cell)
{
  switch(cell)
  {
    case 1:
      return moveUntilLimit(M1_EN, M1_DIR, M1_STEP,
               true,  U1_TOWARD_LIMIT, posM1);
    case 2:
      return moveUntilLimit(M1_EN, M1_DIR, M1_STEP,
               false, U1_AWAY_LIMIT,   posM1);
    case 3:
      return moveUntilLimit(M2_EN, M2_DIR, M2_STEP,
               true,  U2_TOWARD_LIMIT, posM2);
    case 4:
      return moveUntilLimit(M2_EN, M2_DIR, M2_STEP,
               false, U2_AWAY_LIMIT,   posM2);
  }
  return false;
}

// =====================================================
// HOME SYSTEM
// =====================================================
void homeSystem()
{
  enableMotor(M1_EN);
  enableMotor(M2_EN);

  while(posM1 != 0)
  {
    bool toward = (posM1 < 0);
    digitalWrite(M1_DIR, toward ? DIR_TOWARD : DIR_AWAY);
    stepMotor(M1_STEP, toward, posM1);
  }

  while(posM2 != 0)
  {
    bool toward = (posM2 < 0);
    digitalWrite(M2_DIR, toward ? DIR_TOWARD : DIR_AWAY);
    stepMotor(M2_STEP, toward, posM2);
  }

  disableMotor(M1_EN);
  disableMotor(M2_EN);

  lc1Active = lc2Active = lc3Active = lc4Active = true;
  lc1Removed = lc2Removed = lc3Removed = lc4Removed = false;
  fail1 = fail2 = fail3 = fail4 = 0;
  stuck1 = stuck2 = stuck3 = stuck4 = 0;

  allLEDsOff();
}

// =====================================================
// HANDLE FAULT
// =====================================================
void handleFault(int cell)
{
  showFaultLED(cell);

  switch(cell)
  {
    case 1:
      if(lc1Removed) return;
      lc1Active = false; lc1Removed = true; break;
    case 2:
      if(lc2Removed) return;
      lc2Active = false; lc2Removed = true; break;
    case 3:
      if(lc3Removed) return;
      lc3Active = false; lc3Removed = true; break;
    case 4:
      if(lc4Removed) return;
      lc4Active = false; lc4Removed = true; break;
  }

  removeLoadCell(cell);
}

// =====================================================
// SAFE READ
// =====================================================
float safeRead(
  HX711 &scale, int cell,
  int &failCount, int &stuckCount,
  float &lastValue, bool active)
{
  if(!active) return 0;

  if(!scale.is_ready())
  {
    failCount++;
    if(failCount >= FAIL_LIMIT) handleFault(cell);
    return lastValue;
  }

  float val = scale.get_units(1);

  if(isnan(val) || abs(val) > 100000)
  {
    failCount++;
    if(failCount >= FAIL_LIMIT) handleFault(cell);
    return lastValue;
  }

  failCount = 0;

  if(abs(val - lastValue) < 0.01f)
    stuckCount++;
  else
    stuckCount = 0;

  if(stuckCount >= STUCK_LIMIT)
    handleFault(cell);

  lastValue = val;
  return val;
}

// =====================================================
// READ ALL CELLS + COMPUTE TOTAL
// =====================================================
void readCells()
{
  w1 = safeRead(scale1, 1, fail1, stuck1, lastW1, lc1Active);
  w2 = safeRead(scale2, 2, fail2, stuck2, lastW2, lc2Active);
  w3 = safeRead(scale3, 3, fail3, stuck3, lastW3, lc3Active);
  w4 = safeRead(scale4, 4, fail4, stuck4, lastW4, lc4Active);

  totalWeight = 0;
  if(lc1Active) totalWeight += w1;
  if(lc2Active) totalWeight += w2;
  if(lc3Active) totalWeight += w3;
  if(lc4Active) totalWeight += w4;

  if(hopperRecorded)
  {
    teaWeight = totalWeight - hopperWeight;
    if(teaWeight < 0) teaWeight = 0;
  }
  else
  {
    teaWeight = totalWeight;
  }
}

// =====================================================
// SEND TOTAL WEIGHT OVER SERIAL
// Slave Board 02 reads this via UART RX
// Format: "TOTAL:49.43\n"
// =====================================================
void sendTotal()
{
  if(millis() - lastSendTime < SEND_INTERVAL_MS) return;
  lastSendTime = millis();

  // Send tea weight (hopper-subtracted) if recorded,
  // otherwise send raw total
  float valueToSend = hopperRecorded ? teaWeight : totalWeight;

  Serial.print("TOTAL:");
  Serial.println(valueToSend, 2);
}

// =====================================================
// SERIAL COMMANDS
// =====================================================
void handleSerial()
{
  if(!Serial.available()) return;

  String cmd = Serial.readStringUntil('\n');
  cmd.trim();

  // K = hopper placed
  if(cmd == "K")
  {
    hopperPlaced = true;
  }

  // W = record hopper weight
  else if(cmd == "W")
  {
    if(!hopperPlaced) return;
    hopperWeight   = totalWeight;
    hopperRecorded = true;
  }

  // H = home
  else if(cmd == "H")
  {
    homeSystem();
  }

  // R = full reset
  else if(cmd == "R")
  {
    hopperPlaced   = false;
    hopperRecorded = false;
    hopperWeight   = 0;
    homeSystem();
  }
}

// =====================================================
// SETUP
// =====================================================
void setup()
{
  Serial.begin(115200);

  // LEDs
  pinMode(LED1, OUTPUT); pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT); pinMode(LED4, OUTPUT);

  // Motors
  pinMode(M1_STEP, OUTPUT); pinMode(M1_DIR, OUTPUT); pinMode(M1_EN, OUTPUT);
  pinMode(M2_STEP, OUTPUT); pinMode(M2_DIR, OUTPUT); pinMode(M2_EN, OUTPUT);

  // Limit switches
  pinMode(U1_TOWARD_LIMIT, INPUT_PULLUP);
  pinMode(U1_AWAY_LIMIT,   INPUT_PULLUP);
  pinMode(U2_TOWARD_LIMIT, INPUT_PULLUP);
  pinMode(U2_AWAY_LIMIT,   INPUT_PULLUP);

  disableMotor(M1_EN);
  disableMotor(M2_EN);

  // HX711
  scale1.begin(HX1_DT, HX1_SCK); scale1.set_scale(cal1);
  scale2.begin(HX2_DT, HX2_SCK); scale2.set_scale(cal2);
  scale3.begin(HX3_DT, HX3_SCK); scale3.set_scale(cal3);
  scale4.begin(HX4_DT, HX4_SCK); scale4.set_scale(cal4);
}

// =====================================================
// LOOP
// =====================================================
void loop()
{
  handleSerial();
  readCells();
  sendTotal();
}
