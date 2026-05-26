# 🍵 Tea Measure System

An automated **tea leaf weighing and dispensing system** built on a **dual-board architecture** using Arduino Mega and ESP32-S3 microcontrollers. The system uses HX711 load cells for precision weight measurement and stepper motors to control hopper gates, enabling accurate dispensing of tea to target weights across three independent channels.

---

## System Architecture

```
┌─────────────────────────────────────┐
│          SLAVE BOARD 01             │
│        (Arduino Mega)               │
│                                     │
│  4× HX711 Load Cells (scale 1–4)   │
│  2× NEMA Stepper Motors            │
│  6× Limit Switches                 │
│  4× Fault LEDs                     │
│                                     │
│  Outputs: "TOTAL:<weight>\n"        │
│           via Serial TX (UART)      │
└───────────────┬─────────────────────┘
                │  UART (115200 baud)
                │  TX ──► RX (GPIO 16)
                ▼
┌─────────────────────────────────────┐
│          SLAVE BOARD 02             │
│        (ESP32-S3 DevKit)            │
│                                     │
│  2× HX711 Load Cells (scale 5–6)   │
│  3× NEMA17 Steppers (TB6600)       │
│  6× Limit Switches                 │
│                                     │
│  Motor 1 ◄── Slave01 total weight   │
│  Motor 2 ◄── Local cell 5           │
│  Motor 3 ◄── Local cell 6           │
└─────────────────────────────────────┘
```

---

## Features

- **6 HX711 Load Cells** — 4 on Board 01 (combined platform scale), 2 on Board 02 (individual hopper scales)
- **5 Stepper Motors** — 2 on Board 01 (load-cell retraction), 3 on Board 02 (hopper gate control via TB6600 drivers)
- **Fault-Tolerant Weighing** — Board 01 detects stuck/failed cells and physically retracts them with motors while continuing to weigh with the remaining cells (min 3-of-4)
- **Automatic Hopper Tare** — Record and subtract hopper weight from the total to report net tea weight
- **Target-Based Dispensing** — Board 02 runs 3 independent state machines that open hopper gates, monitor weight, reduce flow at 95%, and stop at 100%
- **Non-Blocking Design** — All motor stepping and sensor polling is non-blocking for real-time responsiveness
- **Emergency Stop** — Instantly halt all motors and disable drivers

---

## Hardware Requirements

### Slave Board 01 — Arduino Mega

| Component           | Quantity | Notes                          |
|---------------------|----------|--------------------------------|
| Arduino Mega 2560   | 1        | Or compatible                  |
| HX711 Amplifier     | 4        | One per load cell              |
| Load Cell           | 4        | Platform-style, combined       |
| NEMA Stepper Motor  | 2        | For load-cell retraction       |
| Stepper Driver      | 2        | Step/Dir/Enable interface      |
| Limit Switch        | 4        | 2 per motor (toward / away)    |
| LED                 | 4        | Fault indicators per cell      |

### Slave Board 02 — ESP32-S3

| Component           | Quantity | Notes                          |
|---------------------|----------|--------------------------------|
| ESP32-S3 DevKit     | 1        | Standard variant               |
| HX711 Amplifier     | 2        | One per load cell              |
| Load Cell           | 2        | Individual hopper scales       |
| NEMA17 Stepper      | 3        | Hopper gate control            |
| TB6600 Driver       | 3        | PUL/DIR/EN interface           |
| Limit Switch        | 6        | Upper + Lower per motor        |

---

## Pin Assignments

### Slave Board 01 (Arduino Mega)

<details>
<summary>Click to expand pin table</summary>

| Function           | Pin(s)       |
|--------------------|-------------|
| HX711 #1 DT/SCK   | 8 / 9       |
| HX711 #2 DT/SCK   | 6 / 7       |
| HX711 #3 DT/SCK   | 4 / 5       |
| HX711 #4 DT/SCK   | 10 / 11     |
| LEDs 1–4           | 12, 13, 14, 15 |
| Motor 1 STEP/DIR/EN| 16 / 17 / 18 |
| Motor 2 STEP/DIR/EN| 19 / 20 / 21 |
| M1 Toward Limit    | 1            |
| M1 Away Limit      | 2            |
| M2 Toward Limit    | 42           |
| M2 Away Limit      | 41           |

</details>

### Slave Board 02 (ESP32-S3)

<details>
<summary>Click to expand pin table</summary>

| Function             | Pin(s)        |
|----------------------|--------------|
| UART2 RX (from SB01) | GPIO 16      |
| UART2 TX (unused)    | GPIO 17      |
| HX711 #5 DT/SCK     | 4 / 5        |
| HX711 #6 DT/SCK     | 6 / 7        |
| Motor 1 PUL/DIR/EN   | 38 / 39 / 40 |
| Motor 1 Upper/Lower  | 41 / 42      |
| Motor 2 PUL/DIR/EN   | 35 / 36 / 37 |
| Motor 2 Upper/Lower  | 15 / 16      |
| Motor 3 PUL/DIR/EN   | 11 / 12 / 13 |
| Motor 3 Upper/Lower  | 9 / 10       |

</details>

---

## Wiring Between Boards

| Slave Board 01 | → | Slave Board 02    |
|----------------|---|-------------------|
| Serial TX      | → | GPIO 16 (UART2 RX)|
| GND            | → | GND               |

> **Note:** Only a one-way UART link is needed. Board 01 transmits weight data; Board 02 receives it.

---

## Serial Commands

### Slave Board 01 — USB Serial (115200 baud)

| Command | Description                                              |
|---------|----------------------------------------------------------|
| `K`     | Mark hopper as placed on the scale                       |
| `W`     | Record current total as hopper weight (tare for hopper)  |
| `H`     | Home system — return all retracted cells to active       |
| `R`     | Full reset — clear hopper data and home all cells        |

### Slave Board 02 — USB Serial (115200 baud)

| Command       | Description                                   |
|---------------|-----------------------------------------------|
| `T1:<value>`  | Set target weight (grams) for Motor 1 channel |
| `T2:<value>`  | Set target weight (grams) for Motor 2 channel |
| `T3:<value>`  | Set target weight (grams) for Motor 3 channel |
| `START`       | Home all motors, then descend and begin dispensing |
| `STOP`        | Emergency stop — disable all motors immediately |
| `HOME`        | Move all motors to upper limit switch         |
| `STATUS`      | Print current weights, targets, and motor states |

---

## How It Works

### Slave Board 01 — Weight Measurement

1. **Reads 4 HX711 load cells** every loop iteration with individual calibration factors
2. **Detects faults** via two mechanisms:
   - **Communication failure** — HX711 not ready or returns NaN/overflow → increments fail counter → triggers fault after 2 consecutive failures
   - **Stuck detection** — weight value unchanged (±0.01) for 10 consecutive reads → triggers fault
3. **On fault:** lights the corresponding LED, marks the cell inactive, and **physically retracts** it using a stepper motor (moving to a limit switch)
4. **Computes total weight** from all remaining active cells (minimum 3 of 4)
5. **Hopper tare:** when commanded, stores the current total as hopper weight and subtracts it from subsequent readings
6. **Outputs** `TOTAL:<weight>` over Serial every 300 ms

### Slave Board 02 — Dispensing Control

Each of the 3 motor channels operates an **independent state machine**:

```
  IDLE ──► HOMING ──► DESCENDING ──► MONITORING
                                        │
                              ┌─────────┤
                              ▼         ▼
                          at 95%    at 100%
                              │         │
                              ▼         ▼
                          BACKOFF   GOING_DONE ──► DONE
                              │
                              ▼
                          MONITORING (resume)
```

| State        | Behavior                                                    |
|--------------|-------------------------------------------------------------|
| `IDLE`       | Motor disabled, waiting for `START`                         |
| `HOMING`     | Moving UP until upper limit switch is triggered             |
| `DESCENDING` | Moving DOWN until lower limit switch (gate fully open)      |
| `MONITORING` | Watching weight vs. target                                  |
| `BACKOFF`    | At 95% — steps UP by 200 steps to partially close the gate |
| `GOING_DONE` | At 100% — moving UP to upper limit to fully close gate     |
| `DONE`       | Cycle complete, motor disabled                              |
| `ESTOP`      | Emergency stop, motor disabled                              |

---

## Calibration

### Load Cell Calibration Factors

| Cell    | Board    | Default Factor | Variable |
|---------|----------|---------------|----------|
| Scale 1 | Board 01 | 639.21        | `cal1`   |
| Scale 2 | Board 01 | 635.30        | `cal2`   |
| Scale 3 | Board 01 | 654.45        | `cal3`   |
| Scale 4 | Board 01 | 635.85        | `cal4`   |
| Scale 5 | Board 02 | 639.21        | `cal5`   |
| Scale 6 | Board 02 | 635.30        | `cal6`   |

To calibrate: place a known weight, adjust the calibration factor until `get_units()` returns the correct value.

### Motor Tuning Parameters

| Parameter         | Board    | Default | Description                        |
|-------------------|----------|---------|------------------------------------|
| `STEP_DELAY_US`   | Board 01 | 60 µs   | Delay between steps                |
| `STEP_PULSE_US`   | Board 01 | 5 µs    | Step pulse width                   |
| `PUL_HIGH_US`     | Board 02 | 10 µs   | TB6600 pulse high time             |
| `PUL_PERIOD_US`   | Board 02 | 200 µs  | Total step period                  |
| `BACKOFF_STEPS`   | Board 02 | 200     | Steps to partially close at 95%    |

---

## Quick Start

1. **Wire up** all load cells, motors, limit switches, and LEDs per the pin tables above
2. **Connect boards** via UART (Board 01 TX → Board 02 GPIO 16 RX, plus shared GND)
3. **Flash `Slave_Board_01.ino`** to the Arduino Mega
4. **Flash `Slave_Board_02.ino`** to the ESP32-S3
5. **Open Board 02's Serial Monitor** at 115200 baud
6. **Set targets:**
   ```
   T1:50.00
   T2:30.00
   T3:25.00
   ```
7. **Start dispensing:**
   ```
   START
   ```
8. **Monitor progress:**
   ```
   STATUS
   ```

---

## Dependencies

- [HX711 Arduino Library](https://github.com/bogde/HX711) — Install via Arduino Library Manager
- Arduino IDE or PlatformIO with board support for:
  - Arduino Mega 2560
  - ESP32-S3 (via [arduino-esp32](https://github.com/espressif/arduino-esp32))

---

## Project Structure

```
tea-measure-system/
├── Slave_Board_01.ino   # Arduino Mega — 4× load cells, 2× motors, fault detection
├── Slave_Board_02.ino   # ESP32-S3 — 2× load cells, 3× motors, dispensing state machine
├── LICENSE              # MIT License
└── README.md            # This file
```

---

## License

This project is licensed under the **MIT License** — see the [LICENSE](LICENSE) file for details.

© 2026 Dumindu Dilshan