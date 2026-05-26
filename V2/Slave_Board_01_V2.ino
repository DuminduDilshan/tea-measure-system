// =====================================================
// SLAVE BOARD 01  —  V2
// ESP32-S3 DevKit
//
// LIMIT SWITCH LAYOUT:
//   Motor 1: U1_TOWARD (pin 1) = home + fault retract
//            U1_AWAY   (pin 2) = safety end-stop only
//   Motor 2: U2_TOWARD (pin 42) = home + fault retract
//            U2_AWAY   (pin 41) = safety end-stop only
//
// HOMING SEQUENCE (per motor):
//   1. Drive TOWARD direction until TOWARD limit triggers
//   2. Back AWAY 200 steps  (HOME_PARK_STEPS)
//   3. Reset position counter to 0
//
// AWAY LIMIT behaviour:
//   Triggers hard-stop (motor disabled, warning printed)
//
// FAULT HANDLING:
//   Bad/stuck cell  →  drive motor TOWARD its limit,
//                      disable that cell
//
// COMMANDS (case-insensitive, USB serial OR relayed from Slave02):
//   K / k   Hopper placed
//   W / w   Tare all active cells (zero with hopper on)
//   H / h   Home both motors
//   R / r   Full reset + home
//
// SERIAL OUTPUT (for Slave Board 02 via UART2 RX):
//   "TOTAL:<grams>\n"  every 300 ms
// =====================================================

#include "HX711.h"

// =====================================================
// UART2  —  relay commands received from Slave Board 02
//   GPIO16 = RX  ←  Slave02 TX
//   GPIO17 = TX  (not used here)
// =====================================================
#define SL02_RX_PIN  16
#define SL02_TX_PIN  17

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
#define LED1  12
#define LED2  13
#define LED3  14
#define LED4  15

// =====================================================
// MOTOR 1  (STEP / DIR / EN)
// =====================================================
#define M1_STEP  16
#define M1_DIR   17
#define M1_EN    18

// =====================================================
// MOTOR 2  (STEP / DIR / EN)
// =====================================================
#define M2_STEP  19
#define M2_DIR   20
#define M2_EN    21

// =====================================================
// LIMIT SWITCHES  (INPUT_PULLUP, active LOW)
// Motor 1
#define U1_TOWARD_LIMIT   1   // home + fault retract
#define U1_AWAY_LIMIT     2   // safety end-stop only
// Motor 2
#define U2_TOWARD_LIMIT  42   // home + fault retract
#define U2_AWAY_LIMIT    41   // safety end-stop only
// =====================================================

// =====================================================
// DIRECTION CONVENTION
// =====================================================
#define DIR_TOWARD  HIGH
#define DIR_AWAY    LOW

// =====================================================
// STEP TIMING
// =====================================================
#define STEP_PULSE_US   5    // pulse HIGH duration (µs)
#define STEP_DELAY_US  60    // delay after pulse LOW (µs)

// =====================================================
// HOME PARK OFFSET
// Steps to back AWAY after hitting TOWARD limit
// =====================================================
#define HOME_PARK_STEPS  200

// =====================================================
// HX711 OBJECTS
// =====================================================
HX711 scale1, scale2, scale3, scale4;

// =====================================================
// CALIBRATION FACTORS  (adjust after calibration)
// =====================================================
float cal1 = 639.21f;
float cal2 = 635.30f;
float cal3 = 654.45f;
float cal4 = 635.85f;

// =====================================================
// LOAD CELL STATE FLAGS
// =====================================================
bool lc1Active  = true, lc2Active  = true;
bool lc3Active  = true, lc4Active  = true;

bool lc1Removed = false, lc2Removed = false;
bool lc3Removed = false, lc4Removed = false;

// =====================================================
// HX FAIL + STUCK COUNTERS
// =====================================================
int fail1 = 0, fail2 = 0, fail3 = 0, fail4 = 0;
const int FAIL_LIMIT = 2;

float lastW1 = 0, lastW2 = 0, lastW3 = 0, lastW4 = 0;
int   stuck1 = 0, stuck2 = 0, stuck3 = 0, stuck4 = 0;
const int STUCK_LIMIT = 10;

// =====================================================
// WEIGHT VARIABLES
// =====================================================
float w1 = 0, w2 = 0, w3 = 0, w4 = 0;
float totalWeight = 0;
float teaWeight   = 0;

// =====================================================
// HOPPER STATE
// =====================================================
bool hopperPlaced = false;
bool hopperTared  = false;

// =====================================================
// MOTOR POSITION COUNTERS
// =====================================================
long posM1 = 0;
long posM2 = 0;

// =====================================================
// SERIAL OUTPUT TIMING
// =====================================================
unsigned long lastSendTime = 0;
#define SEND_INTERVAL_MS  300

// =====================================================
// UART2 RECEIVE BUFFER  (relay from Slave02)
// =====================================================
String uart2Buf = "";

// =====================================================
// LED HELPERS
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
// LOW-LEVEL MOTOR HELPERS
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
// CHECK SAFETY (AWAY) LIMITS  (called during moves)
// Returns true if the away limit just triggered →
// caller should abort the move.
// =====================================================
bool checkAwayLimit(int awayLimitPin, int en, const char *motorName)
{
  if(digitalRead(awayLimitPin) == LOW)
  {
    disableMotor(en);
    Serial.print("SAFETY LIMIT HIT: ");
    Serial.print(motorName);
    Serial.println(" — motor disabled");
    return true;   // safety triggered
  }
  return false;
}

// =====================================================
// MOVE TOWARD LIMIT  (blocking, checks away safety)
// =====================================================
void moveTilTowardLimit(
  int en, int dir, int step,
  int towardLimitPin, int awayLimitPin,
  long &pos)
{
  enableMotor(en);
  digitalWrite(dir, DIR_TOWARD);

  while(digitalRead(towardLimitPin) != LOW)
  {
    // Check away safety during inward move
    if(checkAwayLimit(awayLimitPin, en, "M?"))
      return;

    stepMotor(step, true, pos);
  }

  disableMotor(en);
}

// =====================================================
// MOVE AWAY FIXED STEPS  (blocking, checks toward safety)
// =====================================================
void moveAwaySteps(
  int en, int dir, int step,
  int towardLimitPin,
  int numSteps, long &pos)
{
  enableMotor(en);
  digitalWrite(dir, DIR_AWAY);

  for(int i = 0; i < numSteps; i++)
  {
    // If toward limit somehow triggers during park, stop
    if(digitalRead(towardLimitPin) == LOW)
    {
      disableMotor(en);
      return;
    }
    stepMotor(step, false, pos);
  }

  disableMotor(en);
}

// =====================================================
// HOME ONE MOTOR
//   1. Drive TOWARD until TOWARD limit triggers
//   2. Back AWAY HOME_PARK_STEPS steps
//   3. Zero position counter
// =====================================================
void homeOneMotor(
  int en, int dir, int step,
  int towardLimitPin, int awayLimitPin,
  long &pos,
  const char *name)
{
  Serial.print("  Homing ");
  Serial.print(name);
  Serial.println(" ...");

  // Step 1: drive toward limit
  moveTilTowardLimit(en, dir, step,
    towardLimitPin, awayLimitPin, pos);

  Serial.print("  ");
  Serial.print(name);
  Serial.println(" toward-limit hit — parking...");

  // Step 2: back away 200 steps
  moveAwaySteps(en, dir, step,
    towardLimitPin, HOME_PARK_STEPS, pos);

  // Step 3: zero position
  pos = 0;

  Serial.print("  ");
  Serial.print(name);
  Serial.println(" homed & parked at 0");
}

// =====================================================
// HOME BOTH MOTORS
// =====================================================
void homeSystem()
{
  Serial.println("=== HOME START ===");
  allLEDsOff();

  homeOneMotor(M1_EN, M1_DIR, M1_STEP,
    U1_TOWARD_LIMIT, U1_AWAY_LIMIT, posM1, "M1");

  homeOneMotor(M2_EN, M2_DIR, M2_STEP,
    U2_TOWARD_LIMIT, U2_AWAY_LIMIT, posM2, "M2");

  // Reset cell health
  lc1Active   = lc2Active   = lc3Active   = lc4Active   = true;
  lc1Removed  = lc2Removed  = lc3Removed  = lc4Removed  = false;
  fail1 = fail2 = fail3 = fail4 = 0;
  stuck1 = stuck2 = stuck3 = stuck4 = 0;

  allLEDsOff();
  Serial.println("=== HOME COMPLETE ===");
}

// =====================================================
// REMOVE LOAD CELL  (fault retract — drive TOWARD limit)
// Cells 1+2 share Motor 1 → drive to U1_TOWARD_LIMIT
// Cells 3+4 share Motor 2 → drive to U2_TOWARD_LIMIT
// =====================================================
void removeLoadCell(int cell)
{
  Serial.print("REMOVING CELL ");
  Serial.println(cell);

  if(cell == 1 || cell == 2)
  {
    moveTilTowardLimit(
      M1_EN, M1_DIR, M1_STEP,
      U1_TOWARD_LIMIT, U1_AWAY_LIMIT, posM1);
  }
  else if(cell == 3 || cell == 4)
  {
    moveTilTowardLimit(
      M2_EN, M2_DIR, M2_STEP,
      U2_TOWARD_LIMIT, U2_AWAY_LIMIT, posM2);
  }

  Serial.println("CELL RETRACTED");
}

// =====================================================
// HANDLE FAULT
// =====================================================
void handleFault(int cell)
{
  Serial.print("FAULT CELL "); Serial.println(cell);
  showFaultLED(cell);

  switch(cell)
  {
    case 1: if(lc1Removed) return; lc1Active=false; lc1Removed=true; break;
    case 2: if(lc2Removed) return; lc2Active=false; lc2Removed=true; break;
    case 3: if(lc3Removed) return; lc3Active=false; lc3Removed=true; break;
    case 4: if(lc4Removed) return; lc4Active=false; lc4Removed=true; break;
  }

  removeLoadCell(cell);
  Serial.println("SYSTEM REBALANCED");
}

// =====================================================
// SAFE READ HX711
// =====================================================
float safeRead(
  HX711 &scale, int cell,
  int &failCount, int &stuckCount,
  float &lastValue, bool active)
{
  if(!active) return 0.0f;

  if(!scale.is_ready())
  {
    failCount++;
    if(failCount >= FAIL_LIMIT) handleFault(cell);
    return lastValue;
  }

  float val = scale.get_units(1);

  if(isnan(val) || abs(val) > 100000.0f)
  {
    failCount++;
    if(failCount >= FAIL_LIMIT) handleFault(cell);
    return lastValue;
  }

  failCount = 0;

  if(abs(val - lastValue) < 0.01f) stuckCount++;
  else                              stuckCount = 0;

  if(stuckCount >= STUCK_LIMIT) handleFault(cell);

  lastValue = val;
  return val;
}

// =====================================================
// READ ALL CELLS
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

  // After tare, HX711 offsets already applied —
  // teaWeight == totalWeight directly
  teaWeight = (totalWeight < 0) ? 0 : totalWeight;
}

// =====================================================
// TARE ALL ACTIVE CELLS  (W command)
// Zeros the HX711 output with hopper on platform
// =====================================================
void tareAllCells()
{
  Serial.println("TARING cells...");
  if(lc1Active) { scale1.tare(10); Serial.println("  Cell 1 tared"); }
  if(lc2Active) { scale2.tare(10); Serial.println("  Cell 2 tared"); }
  if(lc3Active) { scale3.tare(10); Serial.println("  Cell 3 tared"); }
  if(lc4Active) { scale4.tare(10); Serial.println("  Cell 4 tared"); }

  w1 = w2 = w3 = w4 = 0;
  totalWeight = teaWeight = 0;
  lastW1 = lastW2 = lastW3 = lastW4 = 0;
  hopperTared = true;

  Serial.println("TARE COMPLETE");
}

// =====================================================
// SEND TOTAL WEIGHT TO SLAVE02
// Format: "TOTAL:49.43\n"
// =====================================================
void sendTotal()
{
  if(millis() - lastSendTime < (unsigned long)SEND_INTERVAL_MS) return;
  lastSendTime = millis();

  Serial.print("TOTAL:");
  Serial.println(teaWeight, 2);
}

// =====================================================
// PROCESS ONE COMMAND  (shared by USB + UART relay)
// =====================================================
void processCommand(const String &cmd)
{
  // K / k — hopper placed
  if(cmd.equalsIgnoreCase("K"))
  {
    hopperPlaced = true;
    Serial.println("HOPPER PLACED — send W to tare");
    return;
  }

  // W / w — tare
  if(cmd.equalsIgnoreCase("W"))
  {
    if(!hopperPlaced)
    {
      Serial.println("ERROR: Send K first (hopper not placed)");
      return;
    }
    tareAllCells();
    return;
  }

  // H / h — home
  if(cmd.equalsIgnoreCase("H"))
  {
    homeSystem();
    return;
  }

  // R / r — full reset
  if(cmd.equalsIgnoreCase("R"))
  {
    Serial.println("RESET");
    hopperPlaced = false;
    hopperTared  = false;

    // Re-init scales to clear tare offset
    scale1.begin(HX1_DT, HX1_SCK); scale1.set_scale(cal1);
    scale2.begin(HX2_DT, HX2_SCK); scale2.set_scale(cal2);
    scale3.begin(HX3_DT, HX3_SCK); scale3.set_scale(cal3);
    scale4.begin(HX4_DT, HX4_SCK); scale4.set_scale(cal4);

    homeSystem();
    return;
  }
}

// =====================================================
// HANDLE USB SERIAL
// =====================================================
void handleSerial()
{
  if(!Serial.available()) return;
  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  if(cmd.length() > 0) processCommand(cmd);
}

// =====================================================
// HANDLE UART2  (relay from Slave Board 02)
// =====================================================
void handleUART2()
{
  while(Serial2.available())
  {
    char c = (char)Serial2.read();

    if(c == '\n')
    {
      uart2Buf.trim();
      if(uart2Buf.length() > 0)
      {
        Serial.print("[RELAY from Slave02] ");
        Serial.println(uart2Buf);
        processCommand(uart2Buf);
      }
      uart2Buf = "";
    }
    else if(c != '\r')
    {
      uart2Buf += c;
    }
  }
}

// =====================================================
// POLL AWAY LIMITS IN LOOP  (safety end-stop check)
// Called every loop to catch runaway movement
// =====================================================
void pollSafetyLimits()
{
  // Motor 1 away safety
  if(digitalRead(U1_AWAY_LIMIT) == LOW)
  {
    disableMotor(M1_EN);
    Serial.println("SAFETY: M1 away-limit hit — M1 disabled");
  }

  // Motor 2 away safety
  if(digitalRead(U2_AWAY_LIMIT) == LOW)
  {
    disableMotor(M2_EN);
    Serial.println("SAFETY: M2 away-limit hit — M2 disabled");
  }
}

// =====================================================
// SETUP
// =====================================================
void setup()
{
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, SL02_RX_PIN, SL02_TX_PIN);

  // LEDs
  pinMode(LED1, OUTPUT); pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT); pinMode(LED4, OUTPUT);
  allLEDsOff();

  // Motors
  pinMode(M1_STEP, OUTPUT); pinMode(M1_DIR, OUTPUT); pinMode(M1_EN, OUTPUT);
  pinMode(M2_STEP, OUTPUT); pinMode(M2_DIR, OUTPUT); pinMode(M2_EN, OUTPUT);
  disableMotor(M1_EN);
  disableMotor(M2_EN);

  // Limit switches — INPUT_PULLUP, active LOW
  pinMode(U1_TOWARD_LIMIT, INPUT_PULLUP);
  pinMode(U1_AWAY_LIMIT,   INPUT_PULLUP);
  pinMode(U2_TOWARD_LIMIT, INPUT_PULLUP);
  pinMode(U2_AWAY_LIMIT,   INPUT_PULLUP);

  // HX711
  scale1.begin(HX1_DT, HX1_SCK); scale1.set_scale(cal1);
  scale2.begin(HX2_DT, HX2_SCK); scale2.set_scale(cal2);
  scale3.begin(HX3_DT, HX3_SCK); scale3.set_scale(cal3);
  scale4.begin(HX4_DT, HX4_SCK); scale4.set_scale(cal4);

  Serial.println("==============================");
  Serial.println("  SLAVE BOARD 01  V2");
  Serial.println("==============================");
  Serial.println("  Limit 1+2  →  Motor 1");
  Serial.println("  Limit 3+4  →  Motor 2");
  Serial.println("  Home limit : TOWARD (1, 3)");
  Serial.println("  Safety lim : AWAY   (2, 4)");
  Serial.println("------------------------------");
  Serial.println("Auto-homing on startup...");

  homeSystem();

  Serial.println("------------------------------");
  Serial.println(" K/k  Hopper placed");
  Serial.println(" W/w  Tare cells");
  Serial.println(" H/h  Home motors");
  Serial.println(" R/r  Full reset");
  Serial.println("==============================");
}

// =====================================================
// LOOP
// =====================================================
void loop()
{
  handleSerial();
  handleUART2();
  pollSafetyLimits();
  readCells();
  sendTotal();
}
