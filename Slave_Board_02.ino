// =====================================================
// SLAVE BOARD 02
// ESP32-S3 DevKit (standard)
//
// - Receives TOTAL weight from Slave Board 01 via UART2
//   Format expected: "TOTAL:49.43\n"
// - 2x HX711 local load cells (scale5, scale6)
// - 3x NEMA17 via TB6600 (PUL/DIR/EN)
// - 6x limit switches (upper + lower per motor)
//
// MOTOR ASSIGNMENT:
//   Motor 1  <--  Slave01 total (received via UART2)
//   Motor 2  <--  Local HX711 cell 5
//   Motor 3  <--  Local HX711 cell 6
//
// MOTOR FLOW per channel (fully independent):
//   1. HOME      : move UP to upper limit switch
//   2. DESCEND   : move DOWN to lower limit switch
//   3. MONITOR   : watch weight vs target
//   4. 95% hit   : step UP by BACKOFF_STEPS (reduce flow)
//                  resume monitoring after backoff
//   5. 100% hit  : move UP to upper limit switch (stop flow)
//   6. DONE      : wait for next START
//
// SERIAL COMMANDS (USB Serial Monitor 115200):
//   T1:<val>   Set target weight for Motor 1
//   T2:<val>   Set target weight for Motor 2
//   T3:<val>   Set target weight for Motor 3
//   START      Home all motors then descend and run
//   STOP       Emergency stop all motors
//   HOME       Move all motors to upper limit
//   STATUS     Print weights, targets, motor states
// =====================================================

#include "HX711.h"

// =====================================================
// UART2 — receive from Slave Board 01
// Connect Slave01 Serial TX  -->  ESP32-S3 GPIO16 (RX)
// Connect Slave01 GND        -->  ESP32-S3 GND
// =====================================================
#define SLAVE01_RX_PIN  16
#define SLAVE01_TX_PIN  17   // not used for TX to slave01, kept for Serial2 init

// =====================================================
// LOCAL HX711 PINS
// =====================================================
#define HX5_DT   4
#define HX5_SCK  5

#define HX6_DT   6
#define HX6_SCK  7

// =====================================================
// MOTOR 1  (TB6600)
// =====================================================
#define M1_PUL         38
#define M1_DIR         39
#define M1_EN          40
#define M1_UPPER_LIM   41
#define M1_LOWER_LIM   42

// =====================================================
// MOTOR 2  (TB6600)
// =====================================================
#define M2_PUL         35
#define M2_DIR         36
#define M2_EN          37
#define M2_UPPER_LIM   15
#define M2_LOWER_LIM   16

// =====================================================
// MOTOR 3  (TB6600)
// =====================================================
#define M3_PUL         11
#define M3_DIR         12
#define M3_EN          13
#define M3_UPPER_LIM    9
#define M3_LOWER_LIM   10

// =====================================================
// DIRECTION CONVENTION
// If a motor moves the wrong way, swap HIGH/LOW here
// =====================================================
#define DIR_UP   HIGH
#define DIR_DOWN LOW

// =====================================================
// STEP TIMING (microseconds)
// PUL_PERIOD_US = total period between pulses
// Increase to slow motor down, decrease to speed up
// TB6600 minimum pulse width = 2.5us; 10us is safe
// =====================================================
#define PUL_HIGH_US   10
#define PUL_PERIOD_US 200

// =====================================================
// BACKOFF STEPS at 95%
// Number of steps motor moves UP to reduce flow rate
// Tune this for your hopper geometry
// =====================================================
#define BACKOFF_STEPS  200

// =====================================================
// HX711 CALIBRATION  (adjust after calibration)
// =====================================================
float cal5 = 639.21;
float cal6 = 635.30;

// =====================================================
// HX711 OBJECTS
// =====================================================
HX711 scale5;
HX711 scale6;

// =====================================================
// WEIGHT VARIABLES
// =====================================================
float w5          = 0.0f;   // local cell 5  -> Motor 2
float w6          = 0.0f;   // local cell 6  -> Motor 3
float slave01Total = 0.0f;  // received from Slave01 -> Motor 1

// =====================================================
// TARGET WEIGHTS
// =====================================================
float target[3] = {0, 0, 0};  // target[0]=M1, [1]=M2, [2]=M3

// =====================================================
// MOTOR STATE MACHINE
// =====================================================
enum MotorState : uint8_t
{
  ST_IDLE,        // waiting for START
  ST_HOMING,      // moving UP to upper limit switch
  ST_DESCENDING,  // moving DOWN to lower limit switch
  ST_MONITORING,  // watching weight vs target
  ST_BACKOFF,     // stepping UP (backoff) at 95%
  ST_GOING_DONE,  // heading to upper limit after 100%
  ST_DONE,        // cycle complete, at upper limit
  ST_ESTOP        // emergency stop
};

struct MotorChannel
{
  // Hardware pins
  uint8_t pulPin;
  uint8_t dirPin;
  uint8_t enPin;
  uint8_t upperLim;
  uint8_t lowerLim;

  // State
  MotorState state;

  // Weight source pointer (points to slave01Total, w5, or w6)
  float *weightSrc;

  // Backoff counter
  int backoffRemaining;

  // Flag: 95% backoff already performed this cycle
  bool backedOff;

  // Step timing
  unsigned long lastStepUs;
};

MotorChannel ch[3];

// =====================================================
// SYSTEM RUNNING FLAG
// =====================================================
bool systemRunning = false;

// =====================================================
// HX711 READ TIMING
// =====================================================
unsigned long lastHXMs = 0;
#define HX_INTERVAL_MS 200

// =====================================================
// SLAVE01 UART BUFFER
// =====================================================
String uartBuf = "";

// =====================================================
// MOTOR HELPERS
// =====================================================
void motorEnable(MotorChannel &m)
{
  // TB6600 EN active LOW
  digitalWrite(m.enPin, LOW);
}

void motorDisable(MotorChannel &m)
{
  digitalWrite(m.enPin, HIGH);
}

// Non-blocking single step
// Returns true if step was issued
bool tryStep(MotorChannel &m, bool up)
{
  unsigned long now = micros();
  if((now - m.lastStepUs) < (unsigned long)PUL_PERIOD_US)
    return false;

  digitalWrite(m.dirPin, up ? DIR_UP : DIR_DOWN);
  delayMicroseconds(2); // dir setup time

  digitalWrite(m.pulPin, HIGH);
  delayMicroseconds(PUL_HIGH_US);
  digitalWrite(m.pulPin, LOW);

  m.lastStepUs = now;
  return true;
}

// =====================================================
// STATE MACHINE — called every loop() for each channel
// =====================================================
void runChannel(int i)
{
  MotorChannel &m = ch[i];

  switch(m.state)
  {
    // ------------------------------------------------
    case ST_IDLE:
      motorDisable(m);
      break;

    // ------------------------------------------------
    case ST_HOMING:
    {
      bool atUpper = (digitalRead(m.upperLim) == LOW);

      if(atUpper)
      {
        motorDisable(m);

        // After homing: if system running go descend, else idle
        if(systemRunning)
          m.state = ST_DESCENDING;
        else
          m.state = ST_IDLE;

        Serial.print("[M"); Serial.print(i+1);
        Serial.println("] HOMED — upper limit reached");
        break;
      }

      motorEnable(m);
      tryStep(m, true);   // step UP
      break;
    }

    // ------------------------------------------------
    case ST_DESCENDING:
    {
      bool atLower = (digitalRead(m.lowerLim) == LOW);

      if(atLower)
      {
        motorDisable(m);
        m.state      = ST_MONITORING;
        m.backedOff  = false;

        Serial.print("[M"); Serial.print(i+1);
        Serial.println("] AT LOWER LIMIT — monitoring");
        break;
      }

      motorEnable(m);
      tryStep(m, false);  // step DOWN
      break;
    }

    // ------------------------------------------------
    case ST_MONITORING:
    {
      float weight = *(m.weightSrc);
      float tgt    = target[i];

      if(tgt <= 0.0f) break;   // no target set

      float pct = (weight / tgt) * 100.0f;

      // 100% — stop flow, go to upper limit
      if(pct >= 100.0f)
      {
        motorEnable(m);
        m.state = ST_GOING_DONE;

        Serial.print("[M"); Serial.print(i+1);
        Serial.println("] 100% — going to upper limit");
        break;
      }

      // 95% — backoff (only once per cycle)
      if(pct >= 95.0f && !m.backedOff)
      {
        motorEnable(m);
        m.state            = ST_BACKOFF;
        m.backoffRemaining = BACKOFF_STEPS;

        Serial.print("[M"); Serial.print(i+1);
        Serial.println("] 95% — starting backoff");
        break;
      }

      break;
    }

    // ------------------------------------------------
    case ST_BACKOFF:
    {
      if(m.backoffRemaining <= 0)
      {
        motorDisable(m);
        m.state     = ST_MONITORING;
        m.backedOff = true;

        Serial.print("[M"); Serial.print(i+1);
        Serial.println("] backoff done — monitoring");
        break;
      }

      if(tryStep(m, true))   // step UP
        m.backoffRemaining--;

      break;
    }

    // ------------------------------------------------
    case ST_GOING_DONE:
    {
      // Move UP to upper limit — same as homing
      // but transitions to ST_DONE afterward
      bool atUpper = (digitalRead(m.upperLim) == LOW);

      if(atUpper)
      {
        motorDisable(m);
        m.state = ST_DONE;

        Serial.print("[M"); Serial.print(i+1);
        Serial.println("] CYCLE COMPLETE — at upper limit");
        break;
      }

      motorEnable(m);
      tryStep(m, true);   // step UP
      break;
    }

    // ------------------------------------------------
    case ST_DONE:
      motorDisable(m);
      // Wait for new START command
      break;

    // ------------------------------------------------
    case ST_ESTOP:
      motorDisable(m);
      break;
  }
}

// =====================================================
// READ LOCAL HX711  (non-blocking)
// =====================================================
void readLocalCells()
{
  if(millis() - lastHXMs < HX_INTERVAL_MS) return;
  lastHXMs = millis();

  if(scale5.is_ready())
  {
    float v = scale5.get_units(1);
    if(!isnan(v) && abs(v) < 100000.0f) w5 = v;
  }

  if(scale6.is_ready())
  {
    float v = scale6.get_units(1);
    if(!isnan(v) && abs(v) < 100000.0f) w6 = v;
  }
}

// =====================================================
// PARSE SLAVE01 UART LINE
// Expects: "TOTAL:49.43"
// =====================================================
void parseSlave01(String &line)
{
  if(!line.startsWith("TOTAL:")) return;

  float val = line.substring(6).toFloat();

  if(!isnan(val) && val >= 0.0f && val < 100000.0f)
    slave01Total = val;
}

// =====================================================
// READ SLAVE01 UART  (non-blocking)
// =====================================================
void readSlave01()
{
  while(Serial2.available())
  {
    char c = (char)Serial2.read();

    if(c == '\n')
    {
      uartBuf.trim();
      if(uartBuf.length() > 0)
        parseSlave01(uartBuf);
      uartBuf = "";
    }
    else if(c != '\r')
    {
      uartBuf += c;
    }
  }
}

// =====================================================
// SERIAL COMMANDS  (USB Serial Monitor)
// =====================================================
void handleSerial()
{
  if(!Serial.available()) return;

  String cmd = Serial.readStringUntil('\n');
  cmd.trim();

  // T1/T2/T3:<value>
  if(cmd.startsWith("T") && cmd.length() > 2 && cmd[2] == ':')
  {
    int idx = cmd[1] - '1';   // '1'->0, '2'->1, '3'->2

    if(idx >= 0 && idx <= 2)
    {
      target[idx] = cmd.substring(3).toFloat();
      Serial.print("TARGET M"); Serial.print(idx+1);
      Serial.print(" = "); Serial.println(target[idx], 2);
    }
    return;
  }

  // START
  if(cmd == "START")
  {
    if(target[0] <= 0 || target[1] <= 0 || target[2] <= 0)
    {
      Serial.println("ERROR: Set all targets first (T1, T2, T3)");
      return;
    }

    systemRunning = true;

    for(int i = 0; i < 3; i++)
    {
      ch[i].backedOff        = false;
      ch[i].backoffRemaining = 0;
      ch[i].state            = ST_HOMING;
      motorEnable(ch[i]);
    }

    Serial.println("START — homing all motors");
    return;
  }

  // STOP
  if(cmd == "STOP")
  {
    systemRunning = false;

    for(int i = 0; i < 3; i++)
    {
      ch[i].state = ST_ESTOP;
      motorDisable(ch[i]);
    }

    Serial.println("EMERGENCY STOP");
    return;
  }

  // HOME
  if(cmd == "HOME")
  {
    for(int i = 0; i < 3; i++)
    {
      ch[i].backoffRemaining = 0;
      ch[i].state            = ST_HOMING;
      motorEnable(ch[i]);
    }

    Serial.println("HOMING all motors");
    return;
  }

  // STATUS
  if(cmd == "STATUS")
  {
    const char *names[] = {
      "IDLE","HOMING","DESCENDING",
      "MONITORING","BACKOFF","GOING_DONE","DONE","ESTOP"
    };

    Serial.println("===== STATUS =====");
    Serial.print("Slave01 total : "); Serial.println(slave01Total, 2);
    Serial.print("Local W5      : "); Serial.println(w5, 2);
    Serial.print("Local W6      : "); Serial.println(w6, 2);
    Serial.println("------------------");

    for(int i = 0; i < 3; i++)
    {
      float weight = *(ch[i].weightSrc);
      float tgt    = target[i];
      float pct    = (tgt > 0) ? (weight / tgt * 100.0f) : 0;

      Serial.print("M"); Serial.print(i+1);
      Serial.print("  state="); Serial.print(names[(int)ch[i].state]);
      Serial.print("  weight="); Serial.print(weight, 2);
      Serial.print("  target="); Serial.print(tgt, 2);
      Serial.print("  pct="); Serial.print(pct, 1);
      Serial.println("%");
    }

    Serial.print("System running: ");
    Serial.println(systemRunning ? "YES" : "NO");
    Serial.println("==================");
    return;
  }

  Serial.println("Commands: T1/T2/T3:<val>  START  STOP  HOME  STATUS");
}

// =====================================================
// SETUP
// =====================================================
void setup()
{
  Serial.begin(115200);

  // UART2 from Slave Board 01
  Serial2.begin(115200, SERIAL_8N1, SLAVE01_RX_PIN, SLAVE01_TX_PIN);

  // HX711
  scale5.begin(HX5_DT, HX5_SCK); scale5.set_scale(cal5);
  scale6.begin(HX6_DT, HX6_SCK); scale6.set_scale(cal6);

  // Motor channel init
  uint8_t pulPins[3]  = { M1_PUL, M2_PUL, M3_PUL };
  uint8_t dirPins[3]  = { M1_DIR, M2_DIR, M3_DIR };
  uint8_t enPins[3]   = { M1_EN,  M2_EN,  M3_EN  };
  uint8_t upLims[3]   = { M1_UPPER_LIM, M2_UPPER_LIM, M3_UPPER_LIM };
  uint8_t dnLims[3]   = { M1_LOWER_LIM, M2_LOWER_LIM, M3_LOWER_LIM };
  float  *wSrcs[3]    = { &slave01Total, &w5, &w6 };

  for(int i = 0; i < 3; i++)
  {
    ch[i].pulPin           = pulPins[i];
    ch[i].dirPin           = dirPins[i];
    ch[i].enPin            = enPins[i];
    ch[i].upperLim         = upLims[i];
    ch[i].lowerLim         = dnLims[i];
    ch[i].state            = ST_IDLE;
    ch[i].weightSrc        = wSrcs[i];
    ch[i].backoffRemaining = 0;
    ch[i].backedOff        = false;
    ch[i].lastStepUs       = 0;

    pinMode(pulPins[i], OUTPUT); digitalWrite(pulPins[i], LOW);
    pinMode(dirPins[i], OUTPUT); digitalWrite(dirPins[i], LOW);
    pinMode(enPins[i],  OUTPUT);
    motorDisable(ch[i]);
  }

  // Limit switches — INPUT_PULLUP, active LOW
  uint8_t allLims[6] = {
    M1_UPPER_LIM, M1_LOWER_LIM,
    M2_UPPER_LIM, M2_LOWER_LIM,
    M3_UPPER_LIM, M3_LOWER_LIM
  };
  for(int i = 0; i < 6; i++)
    pinMode(allLims[i], INPUT_PULLUP);

  Serial.println("==============================");
  Serial.println("   SLAVE BOARD 02 READY");
  Serial.println("==============================");
  Serial.println(" T1/T2/T3:<val>  set targets");
  Serial.println(" START           run all motors");
  Serial.println(" STOP            emergency stop");
  Serial.println(" HOME            home all motors");
  Serial.println(" STATUS          print all info");
  Serial.println("==============================");
}

// =====================================================
// LOOP
// =====================================================
void loop()
{
  handleSerial();    // USB serial commands
  readSlave01();     // UART2 weight from Slave01
  readLocalCells();  // HX711 poll

  // Run all 3 motor state machines independently
  runChannel(0);
  runChannel(1);
  runChannel(2);
}
