// =====================================================
// SLAVE BOARD 02  —  V2
// ESP32-S3 DevKit
//
// LIMIT SWITCH LAYOUT (per motor channel):
//   Each motor has TWO limits:
//     UPPER_LIM = home limit  (motor drives UP to this)
//     LOWER_LIM = safety end-stop  (hard stop if hit)
//
// HOMING SEQUENCE (per motor channel):
//   1. Drive UP until UPPER_LIM triggers
//   2. Back DOWN HOME_PARK_STEPS (200) steps
//   3. State → ST_DESCENDING (if systemRunning)
//              or ST_IDLE
//
// LOWER_LIM behaviour:
//   Any time LOWER_LIM is triggered during normal operation
//   → motor disabled immediately (hard stop, warning logged)
//   NOTE: Lower limit is also the descent end-of-travel
//         when in ST_DESCENDING → normal transition to MONITORING
//
// MOTOR ASSIGNMENT:
//   Motor 1  ←  Slave01 tea weight  (UART2 RX)
//   Motor 2  ←  Local HX711 cell 5
//   Motor 3  ←  Local HX711 cell 6
//
// UART2 WIRING:
//   GPIO16 RX  ←  Slave01 TX   (receive TOTAL:<val>)
//   GPIO17 TX  →  Slave01 RX   (relay K/k, W/w commands)
//
// USB SERIAL COMMANDS (115200 baud):
//   K / k         Relay hopper-placed to Slave01
//   W / w         Relay tare command to Slave01
//   T1:<val>      Set target for Motor 1  (grams)
//   T2:<val>      Set target for Motor 2
//   T3:<val>      Set target for Motor 3
//   START         Home all motors then descend & monitor
//   STOP          Emergency stop all motors
//   HOME          Home all motors (manual)
//   STATUS        Print weights, targets, motor states
// =====================================================

#include "HX711.h"

// =====================================================
// UART2  —  Slave Board 01
// =====================================================
#define SLAVE01_RX_PIN  16   // RX: receive TOTAL:<val>
#define SLAVE01_TX_PIN  17   // TX: relay K/W commands

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
#define M1_PUL        38
#define M1_DIR        39
#define M1_EN         40
#define M1_UPPER_LIM  41   // home limit
#define M1_LOWER_LIM  42   // safety end-stop / descent end

// =====================================================
// MOTOR 2  (TB6600)
// =====================================================
#define M2_PUL        35
#define M2_DIR        36
#define M2_EN         37
#define M2_UPPER_LIM  15   // home limit
#define M2_LOWER_LIM  16   // safety end-stop / descent end

// =====================================================
// MOTOR 3  (TB6600)
// =====================================================
#define M3_PUL        11
#define M3_DIR        12
#define M3_EN         13
#define M3_UPPER_LIM   9   // home limit
#define M3_LOWER_LIM  10   // safety end-stop / descent end

// =====================================================
// DIRECTION CONVENTION
// Swap HIGH/LOW here if a motor runs the wrong way
// =====================================================
#define DIR_UP   HIGH
#define DIR_DOWN LOW

// =====================================================
// STEP TIMING  (microseconds)
// =====================================================
#define PUL_HIGH_US    10
#define PUL_PERIOD_US  200

// =====================================================
// BACKOFF STEPS  (at 95% target)
// Motor steps UP to reduce flow rate
// =====================================================
#define BACKOFF_STEPS  200

// =====================================================
// HOME PARK OFFSET
// Steps to back DOWN after upper limit triggers
// =====================================================
#define HOME_PARK_STEPS  200

// =====================================================
// HX711 CALIBRATION
// =====================================================
float cal5 = 639.21f;
float cal6 = 635.30f;

HX711 scale5, scale6;

// =====================================================
// WEIGHT VARIABLES
// =====================================================
float w5           = 0.0f;   // cell 5  → Motor 2
float w6           = 0.0f;   // cell 6  → Motor 3
float slave01Total = 0.0f;   // from Slave01 → Motor 1

// =====================================================
// TARGET WEIGHTS  (grams)
// =====================================================
float target[3] = {0, 0, 0};  // [0]=M1  [1]=M2  [2]=M3

// =====================================================
// MOTOR STATE MACHINE
// =====================================================
enum MotorState : uint8_t
{
  ST_IDLE,        // waiting for START
  ST_HOMING,      // driving UP to upper limit
  ST_PARK,        // backing DOWN after upper limit (park phase)
  ST_DESCENDING,  // driving DOWN to lower limit
  ST_MONITORING,  // watching weight vs target
  ST_BACKOFF,     // stepping UP at 95%
  ST_GOING_DONE,  // heading UP to upper limit at 100%
  ST_DONE,        // cycle complete
  ST_ESTOP        // emergency stop
};

// Human-readable state names (for STATUS command)
const char *STATE_NAMES[] = {
  "IDLE", "HOMING", "PARK", "DESCENDING",
  "MONITORING", "BACKOFF", "GOING_DONE", "DONE", "ESTOP"
};

struct MotorChannel
{
  uint8_t    pulPin;
  uint8_t    dirPin;
  uint8_t    enPin;
  uint8_t    upperLim;   // home limit (UP direction)
  uint8_t    lowerLim;   // safety end-stop / descent end

  MotorState state;
  float     *weightSrc;

  int        backoffRemaining;
  bool       backedOff;
  int        parkRemaining;      // steps left in park phase

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
#define HX_INTERVAL_MS  200

// =====================================================
// SLAVE01 UART BUFFER
// =====================================================
String uartBuf = "";

// =====================================================
// MOTOR HELPERS
// =====================================================
void motorEnable(MotorChannel &m)  { digitalWrite(m.enPin, LOW);  }
void motorDisable(MotorChannel &m) { digitalWrite(m.enPin, HIGH); }

// Non-blocking single step; returns true if step issued
bool tryStep(MotorChannel &m, bool up)
{
  unsigned long now = micros();
  if((now - m.lastStepUs) < (unsigned long)PUL_PERIOD_US)
    return false;

  digitalWrite(m.dirPin, up ? DIR_UP : DIR_DOWN);
  delayMicroseconds(2);   // dir setup time

  digitalWrite(m.pulPin, HIGH);
  delayMicroseconds(PUL_HIGH_US);
  digitalWrite(m.pulPin, LOW);

  m.lastStepUs = now;
  return true;
}

// =====================================================
// STATE MACHINE  —  called every loop() per channel
// =====================================================
void runChannel(int i)
{
  MotorChannel &m = ch[i];

  // -----------------------------------------------
  // SAFETY: lower limit hard-stop
  // (except during ST_DESCENDING descent end detection)
  // -----------------------------------------------
  if(m.state != ST_DESCENDING &&
     m.state != ST_IDLE       &&
     m.state != ST_DONE       &&
     m.state != ST_ESTOP)
  {
    if(digitalRead(m.lowerLim) == LOW)
    {
      motorDisable(m);
      m.state = ST_ESTOP;
      Serial.print("[M"); Serial.print(i+1);
      Serial.println("] SAFETY: lower limit hit — E-STOP");
      return;
    }
  }

  switch(m.state)
  {
    // ------------------------------------------------
    case ST_IDLE:
      motorDisable(m);
      break;

    // ------------------------------------------------
    // Step 1 of homing: drive UP to upper limit
    case ST_HOMING:
    {
      if(digitalRead(m.upperLim) == LOW)
      {
        // Upper limit reached — enter park phase
        m.parkRemaining = HOME_PARK_STEPS;
        m.state         = ST_PARK;

        Serial.print("[M"); Serial.print(i+1);
        Serial.println("] upper limit hit — parking...");
        break;
      }

      motorEnable(m);
      tryStep(m, true);   // step UP
      break;
    }

    // ------------------------------------------------
    // Step 2 of homing: back DOWN HOME_PARK_STEPS
    case ST_PARK:
    {
      if(m.parkRemaining <= 0)
      {
        motorDisable(m);
        m.state = systemRunning ? ST_DESCENDING : ST_IDLE;

        Serial.print("[M"); Serial.print(i+1);
        Serial.println("] HOMED & PARKED");
        break;
      }

      motorEnable(m);
      if(tryStep(m, false))   // step DOWN
        m.parkRemaining--;

      break;
    }

    // ------------------------------------------------
    // Descend to lower limit (normal flow start)
    case ST_DESCENDING:
    {
      if(digitalRead(m.lowerLim) == LOW)
      {
        motorDisable(m);
        m.state     = ST_MONITORING;
        m.backedOff = false;

        Serial.print("[M"); Serial.print(i+1);
        Serial.println("] lower limit — monitoring");
        break;
      }

      motorEnable(m);
      tryStep(m, false);   // step DOWN
      break;
    }

    // ------------------------------------------------
    case ST_MONITORING:
    {
      float weight = *(m.weightSrc);
      float tgt    = target[i];

      if(tgt <= 0.0f) break;   // no target set yet

      float pct = (weight / tgt) * 100.0f;

      // 100% — close valve, drive to upper limit
      if(pct >= 100.0f)
      {
        motorEnable(m);
        m.state = ST_GOING_DONE;

        Serial.print("[M"); Serial.print(i+1);
        Serial.println("] 100% — closing");
        break;
      }

      // 95% — backoff once per cycle
      if(pct >= 95.0f && !m.backedOff)
      {
        motorEnable(m);
        m.state            = ST_BACKOFF;
        m.backoffRemaining = BACKOFF_STEPS;

        Serial.print("[M"); Serial.print(i+1);
        Serial.println("] 95% — backoff");
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
        Serial.println("] backoff done");
        break;
      }

      if(tryStep(m, true))   // step UP (reduce flow)
        m.backoffRemaining--;

      break;
    }

    // ------------------------------------------------
    // Drive UP to upper limit to fully close valve
    case ST_GOING_DONE:
    {
      if(digitalRead(m.upperLim) == LOW)
      {
        motorDisable(m);
        m.state = ST_DONE;

        Serial.print("[M"); Serial.print(i+1);
        Serial.println("] CYCLE COMPLETE");
        break;
      }

      motorEnable(m);
      tryStep(m, true);   // step UP
      break;
    }

    // ------------------------------------------------
    case ST_DONE:
      motorDisable(m);
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
  if(millis() - lastHXMs < (unsigned long)HX_INTERVAL_MS) return;
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
// PARSE SLAVE01 UART LINE  ("TOTAL:49.43")
// =====================================================
void parseSlave01(const String &line)
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
      if(uartBuf.length() > 0) parseSlave01(uartBuf);
      uartBuf = "";
    }
    else if(c != '\r')
    {
      uartBuf += c;
    }
  }
}

// =====================================================
// RELAY COMMAND TO SLAVE01  (via UART2 TX)
// =====================================================
void relayToSlave01(const String &cmd)
{
  Serial2.println(cmd);
  Serial.print("[RELAY → Slave01] ");
  Serial.println(cmd);
}

// =====================================================
// SERIAL COMMANDS  (USB Serial Monitor)
// =====================================================
void handleSerial()
{
  if(!Serial.available()) return;

  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  if(cmd.length() == 0) return;

  // K / k — relay hopper-placed to Slave01
  if(cmd.equalsIgnoreCase("K"))
  {
    relayToSlave01(cmd);
    Serial.println("Hopper placed — relayed to Slave01");
    return;
  }

  // W / w — relay tare to Slave01
  if(cmd.equalsIgnoreCase("W"))
  {
    relayToSlave01(cmd);
    Serial.println("Tare signal — relayed to Slave01");
    return;
  }

  // T1/T2/T3:<value>
  if(cmd.startsWith("T") && cmd.length() > 2 && cmd[2] == ':')
  {
    int idx = cmd[1] - '1';   // '1'→0  '2'→1  '3'→2
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
      ch[i].parkRemaining    = HOME_PARK_STEPS;
      ch[i].state            = ST_HOMING;
      motorEnable(ch[i]);
    }

    Serial.println("START — homing all motors then descending");
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
      ch[i].parkRemaining    = HOME_PARK_STEPS;
      ch[i].state            = ST_HOMING;
      motorEnable(ch[i]);
    }
    Serial.println("HOMING all motors");
    return;
  }

  // STATUS
  if(cmd == "STATUS")
  {
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
      Serial.print("  state=");  Serial.print(STATE_NAMES[(int)ch[i].state]);
      Serial.print("  weight="); Serial.print(weight, 2);
      Serial.print("g  target="); Serial.print(tgt, 2);
      Serial.print("g  pct=");   Serial.print(pct, 1);
      Serial.println("%");
    }

    Serial.print("System running: ");
    Serial.println(systemRunning ? "YES" : "NO");
    Serial.println("==================");
    return;
  }

  Serial.println("Commands: K  W  T1/T2/T3:<val>  START  STOP  HOME  STATUS");
}

// =====================================================
// SETUP
// =====================================================
void setup()
{
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, SLAVE01_RX_PIN, SLAVE01_TX_PIN);

  // HX711
  scale5.begin(HX5_DT, HX5_SCK); scale5.set_scale(cal5);
  scale6.begin(HX6_DT, HX6_SCK); scale6.set_scale(cal6);

  // Motor channel init
  uint8_t pulPins[3] = { M1_PUL, M2_PUL, M3_PUL };
  uint8_t dirPins[3] = { M1_DIR, M2_DIR, M3_DIR };
  uint8_t enPins[3]  = { M1_EN,  M2_EN,  M3_EN  };
  uint8_t upLims[3]  = { M1_UPPER_LIM, M2_UPPER_LIM, M3_UPPER_LIM };
  uint8_t dnLims[3]  = { M1_LOWER_LIM, M2_LOWER_LIM, M3_LOWER_LIM };
  float  *wSrcs[3]   = { &slave01Total, &w5, &w6 };

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
    ch[i].parkRemaining    = HOME_PARK_STEPS;
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
  Serial.println("  SLAVE BOARD 02  V2");
  Serial.println("==============================");
  Serial.println("  Upper limit = home (per motor)");
  Serial.println("  Lower limit = safety end-stop");
  Serial.println("------------------------------");
  Serial.println(" K / k      Relay hopper placed");
  Serial.println(" W / w      Relay tare to Board01");
  Serial.println(" T1/T2/T3:<val>  Set targets");
  Serial.println(" START      Home & run all motors");
  Serial.println(" STOP       Emergency stop");
  Serial.println(" HOME       Home all motors");
  Serial.println(" STATUS     Print all info");
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

  runChannel(0);
  runChannel(1);
  runChannel(2);
}
