# 🚀 HYDRO-1 Flight Computer — v2.0

> **Avionics system for experimental water-powered rockets.**
> Real-time telemetry · Triple-redundant apogee detection · Wi-Fi ground station · 7-state flight FSM

[![Status](https://img.shields.io/badge/status-In%20Development-yellow)](https://github.com/DonJechu/HydroRocket-Telemetry-System)
[![Platform](https://img.shields.io/badge/platform-ESP32-blue)](https://www.espressif.com/)
[![License](https://img.shields.io/badge/license-MIT-green)](LICENSE)
[![Research](https://img.shields.io/badge/research-EMI%20%2F%20Poynting%20Vector-purple)](docs/research/)

---

## 📋 Table of Contents

1. [Project Overview](#project-overview)
2. [Research Context](#research-context)
3. [Hardware Architecture](#hardware-architecture)
4. [Avionics Bay — Design Evolution](#avionics-bay--design-evolution)
5. [Firmware — Flight State Machine](#firmware--flight-state-machine)
6. [Sensor Processing & Filters](#sensor-processing--filters)
7. [Telemetry System](#telemetry-system)
8. [Repository Structure](#repository-structure)
9. [Build & Flash](#build--flash)
10. [Known Issues & v4 Integration Status](#known-issues--v4-integration-status)

---

## Project Overview

**HYDRO-1** is a flight computer designed for water-powered PET bottle rockets. It is the hardware platform for ongoing research into **EMI mitigation in aerospace wiring harnesses**, using the Poynting Vector framework to validate field-based signal integrity strategies in a real high-vibration environment.

The system provides:
- **Real-time telemetry** over Wi-Fi WebSocket at 10 Hz
- **Triple-redundant apogee detection** (velocity zero-crossing + altitude drop + safety timeout)
- **Hardware IIR pressure filtering** on BMP280 to suppress aerodynamic transients
- **Parachute deployment** via servo at confirmed apogee

> ⚠️ **Status:** The rocket has not yet been launched. Currently in integration and ground-testing phase. This repository documents the complete design process including failures, iterations, and corrections — which is the engineering record.

---

## Research Context

This project serves as the experimental platform for ongoing research on EMI mitigation in aerospace wiring harnesses, using a field-theoretic approach (Poynting Vector framework). Research paper in preparation — to be submitted to IEEE

### Core Thesis

Classical intuition treats wires as "pipes" for energy. This is incorrect. The actual energy carrier is the electromagnetic field surrounding the conductor, described by the **Poynting Vector**:

$$\mathbf{S} = \frac{1}{\mu_0}(\mathbf{E} \times \mathbf{B}) \quad \left[\frac{\text{W}}{\text{m}^2}\right]$$

Electrons drift at mere millimeters per second — governed by the **Drude model**:

$$v_d = \frac{I}{nqA}$$

while energy propagates at near light speed through the *external* field. This has direct consequences for EMI: **crosstalk is not a wire problem, it is a field geometry problem.**

### Why this rocket?

The HYDRO-1 wiring harness is intentionally designed in two configurations:
1. Unshielded parallel runs (power + signal lines adjacent) — baseline EMI scenario
2. Twisted pair + separated routing — field-confined scenario

In-flight telemetry data will be compared between configurations to measure real crosstalk impact on sensor fidelity under high-vibration, high-EMI conditions (motor PWM, servo switching, Wi-Fi RF).

---

## Hardware Architecture

### Component Stack

| Component | Part | Role |
|---|---|---|
| **Microcontroller** | ESP32-DEVKIT-V1 (30-pin) | Main processor, Wi-Fi AP, WebSocket server |
| **IMU** | MPU-6050 | 9-axis accelerometer/gyroscope — configured for ±16G |
| **Barometer** | BMP280 | Altitude via pressure — hardware IIR filter enabled |
| **Power Regulation** | MT3608 Boost Converter | 3.7V LiPo → 5V regulated for ESP32 VIN |
| **Battery** | LiPo 103040 — 3.7V / 1200mAh / 4.44Wh | Flight power supply |
| **Actuator** | Standard servo (GPIO 13) | Parachute deployment mechanism |

### I²C Bus

Both MPU-6050 and BMP280 share the I²C bus:

| Signal | ESP32 Pin |
|---|---|
| SDA | GPIO 21 |
| SCL | GPIO 22 |
| MPU-6050 Address | 0x68 |
| BMP280 Address | 0x76 |

### Power Path

```
LiPo 3.7V ──► MT3608 Boost ──► 5V ──► ESP32 VIN ──► 3.3V LDO ──► Sensors
                (adj. to 5V)
```

> **Schematic:** See [`hardware/Schematic_HidroRocket_v0_2026-04-20.pdf`](hardware/Schematic_HidroRocket_v0_2026-04-20.pdf)

---

## Avionics Bay — Design Evolution

The avionics bay has gone through three major design iterations. Each failure was documented and fed directly into the next version. This is the engineering record.

### Version Comparison

| Parameter | v1 | v2 | v3 | v4 (current) |
|---|---|---|---|---|
| **Height** | 118.5 mm | 118 mm | 102 mm | TBD |
| **Base OD** | 52 mm | 81 mm | 80.9 mm | TBD |
| **Printed Mass** | 33.01 g | 66.62 g | 46.22 g | ~23.92 g* |
| **Pieces** | 1 | 2 | 2 | 1 |
| **PCB mount** | Screwed direct | Slide rail | Slide rail (revised) | TBD |
| **Battery space** | None | Insufficient | In revision | TBD |
| **Fits 1.35L bottle** | ❌ | ❌ | ❌ | ⚠️ Printing |
| **Architecture** | 1 piece | 2 pieces | 2 pieces | 1 piece |
| **Structure** | Solid shell | Rail + shell | Honeycomb panel | Truss / celosía |

> *Masses are slicer estimates (PETG, no supports). Final printed mass will differ.*

---

### v1 — Structural Baseline
 
**Goal:** Minimum viable bay. One-piece design, direct PCB mount.
 
**Result — Failed integration:**
- PCB mounting nuts and standoff contractions caused mechanical interference; the board could not be seated flush
- No dedicated battery compartment — LiPo placement undefined
- Ring dimensions incorrect: OD too small to fit inside a 1.35L Coca-Cola PET bottle (~85mm ID required)
- Single-piece design meant any revision required reprinting the entire part
**Mass:** 33.01 g | **Height:** 118.5 mm | **OD:** 52 mm
 
![AvionicsBay v1 Render](media/Prototype%20Gallery/AvionicsBay_v1_render.png)
 
---
 
### v2 — Rail System
 
**Goal:** Solve PCB access problem. Introduce modularity.
 
**Key change:** Split into two parts — a structural sled that mounts to the bottle, and a PCB carrier plate that slides into a rail. This means PCB changes only require reprinting the carrier plate, not the full bay.
 
**Result — Too heavy:**
- Rail + structural shell geometry increased mass to 66.62 g (+100% vs v1)
- Battery space still insufficient for 103040 LiPo cell (30×40×10 mm)
- OD corrected but not validated against physical bottle measurement
**Mass:** 66.62 g | **Height:** 118 mm | **OD:** 81 mm
 
![AvionicsBay v2 Render](media/Prototype%20Gallery/AvionicsBay_v2_render.png)
 
---
 
### v3 — Mass-Optimized Honeycomb (Current Alpha)
 
**Goal:** Reduce mass below 40g while keeping v2's rail modularity. Introduce honeycomb infill geometry on the main panel.
 
**Key changes:**
- Reduced height by 16 mm (102 mm total)
- Honeycomb cutout pattern on main structural panel (mass reduction + ventilation)
- Maintained two-piece rail system from v2
**Failures documented (v3.0 Alpha):**
1. **Mechanical:** Nut/standoff interference on PCB mount — same root cause as v1, not fully resolved
2. **Volumetric:** Battery compartment still does not accommodate LiPo 103040 (30×40×10 mm) + cable routing (minimum 5–8 mm egress space required)
3. **Geometric:** OD not validated against physical bottle. Nominal 80.9 mm but actual 1.35L bottle ID varies 84–87 mm by batch — must be measured with calipers before fixing in CAD
4. **Structural:** Print fracture during support removal on thin-wall sections (<2 mm) with Tree support configuration on Bambu Lab P2S
**Mass:** 46.22 g | **Height:** 102 mm | **OD:** 80.9 mm
 
![AvionicsBay v3 Render](media/Prototype%20Gallery/AvionicsBay_v3_render.png)

 ### v4 — Single-Piece Truss Architecture (Current)

**Goal:** Eliminate over-engineering. Return to single-piece design with truss/celosía geometry for maximum mass reduction while maintaining structural integrity.

**Key changes:**
- Single-piece design — removes rail interface, assembly complexity, and inter-part tolerance issues
- Truss skeleton replaces solid honeycomb panel — material only where structurally necessary
- Estimated model mass: 23.92 g (slicer, no supports) — best result across all versions

**Status:** Printing — `ABv4.2.1.stl` · Layer 91/555 · Bambu Lab P2S · Est. finish 03:51

**Design rationale:** Previous versions (v2, v3) introduced modularity to solve PCB access — but added mass and complexity. v4 returns to v1's simplicity with v3's mass consciousness. Less is more.

![AvionicsBay v4 Render](media/Prototype%20Gallery/AvionicsBay_v4_render.png)

> ⚠️ Integration results pending print completion.

### PCB — Physical Assembly
 
| Bare board (v1 iteration) | Full assembly with ESP32 + sensors |
|---|---|
| ![PCB bare](media/Prototype%20Gallery/PCB_v1_bare_assembly.jpg) | ![PCB full](media/Prototype%20Gallery/PCB_v2_full_assembly.jpg) |
 
---

## Firmware — Flight State Machine

The flight computer implements a 7-state finite state machine. States `ARMED` and `IGNITION` are defined in the enum for future remote-arming via the ground station dashboard but are not yet active in the switch logic.

```
                    gForce ≥ 2.5G
                   (4 consecutive)
  ┌──────────┐ ─────────────────────► ┌──────────┐
  │ STANDBY  │                        │  ASCENT  │
  └──────────┘                        └──────────┘
                                           │
                          ┌────────────────┼────────────────┐
                          │                │                │
                    vel ≤ 0           alt drops        timeout
                   (5 samples)         1.0m (4s)       12,000 ms
                          │                │                │
                          └────────────────┼────────────────┘
                                           ▼
                                      ┌──────────┐
                                      │  APOGEE  │  servo → 90°
                                      └──────────┘
                                           │
                                           ▼
                                      ┌──────────┐
                                      │ DESCENT  │
                                      └──────────┘
                                           │
                                    alt < 0.75 m
                                           │
                                           ▼
                                      ┌──────────┐
                                      │ LANDING  │
                                      └──────────┘
```

### State Descriptions

| State | ID | Entry Condition | Action |
|---|---|---|---|
| `STANDBY` | 0 | Boot | Transmit telemetry, wait for liftoff |
| `ARMED` | 1 | *(future: ground station command)* | Arm pyro / deployment circuit |
| `IGNITION` | 2 | *(future: remote trigger)* | Log ignition timestamp |
| `ASCENT` | 3 | gForce ≥ 2.5G × 4 samples | Track maxAltitude, evaluate apogee |
| `APOGEE` | 4 | Triple-redundant (see below) | `servo.write(90)` — deploy parachute |
| `DESCENT` | 5 | Post-apogee | Monitor altitude, transmit |
| `LANDING` | 6 | altitude < 0.75 m | Stop flight log (LittleFS — pending) |

### Triple-Redundant Apogee Detection

Apogee is the most safety-critical event. Three independent methods must confirm it:

**Method 1 — Velocity Zero-Crossing (Primary)**

The most reliable indicator from rocketry literature. When vertical velocity transitions from positive to zero or negative:

$$v(t) = v(t-1) \cdot 0.85 + \frac{\Delta h}{\Delta t} \cdot 0.15 \leq 0 \quad \text{(5 consecutive samples)}$$

**Method 2 — Altitude Drop (Backup)**

If the filtered altitude falls more than 1.0 m below the recorded maximum:

$$h_{current} < h_{max} - \Delta h_{apogee} \quad (\Delta h_{apogee} = 1.0\text{ m}, \; 4 \text{ confirmations})$$

**Method 3 — Absolute Timeout (Safety)**

If neither primary nor backup triggers within 12 seconds of liftoff confirmation, the parachute deploys unconditionally. This prevents the rocket from impacting the ground without a deployed chute in case of sensor failure.

---

## Sensor Processing & Filters

### BMP280 — Dual IIR Filtering

Aerodynamic turbulence creates pressure transients that corrupt altitude readings. The BMP280 includes a hardware IIR filter that is enabled at maximum coefficient:

```cpp
bmp.setSampling(
  MODE_NORMAL,
  SAMPLING_X2,   // Temperature oversampling
  SAMPLING_X16,  // Pressure oversampling — maximum resolution
  FILTER_X16,    // Hardware IIR — maximum smoothing
  STANDBY_MS_1   // ~27 ms update rate
);
```

A second software IIR filter is applied in the main loop:

$$h_{filtered}[n] = \alpha \cdot h_{filtered}[n-1] + (1-\alpha) \cdot h_{raw}[n]$$

With $\alpha = 0.8$ for flight (responsive to 15–20 m/s dynamics). Note: this will appear noisy in static indoor testing — that is the correct and expected behavior.

### Pressure Calibration

At boot, 300 pressure samples are averaged over ~3 seconds to establish a local ground-level reference:

$$P_{base} = \frac{1}{300}\sum_{i=1}^{300} P_i$$

This must be performed with the rocket stationary on the launch pad.

### MPU-6050 — G-Force & Orientation

The IMU is configured for the high-G environment of a water rocket launch:

```cpp
mpu.setAccRange(MPU9250_ACC_RANGE_16G);   // ±16G
mpu.setGyrRange(MPU9250_GYRO_RANGE_2000); // ±2000°/s
```

G-force magnitude:

$$G = \sqrt{a_x^2 + a_y^2 + a_z^2}$$

Euler angles (from raw accelerometer, no Kalman filter yet — planned for v3.1 firmware):

$$\text{pitch} = \arctan\!\left(\frac{a_x}{\sqrt{a_y^2 + a_z^2}}\right) \cdot \frac{180°}{\pi}$$

$$\text{roll} = \arctan\!\left(\frac{a_y}{a_z}\right) \cdot \frac{180°}{\pi}$$

---

## Telemetry System

The ESP32 creates a Wi-Fi Access Point (`SSID: GANNET`) and serves a WebSocket server on port 81. The ground station connects and receives JSON packets at 10 Hz:

```json
{
  "alt":      42.3,
  "vel":      12.1,
  "accel":    3.47,
  "pitch":    -2.1,
  "roll":     0.8,
  "yaw":      0.0,
  "temp":     28.4,
  "pressure": 1009.2,
  "phase":    3
}
```

| Field | Unit | Description |
|---|---|---|
| `alt` | m | Filtered altitude above launch point |
| `vel` | m/s | Vertical velocity (positive = ascending) |
| `accel` | G | Total acceleration magnitude |
| `pitch` | ° | Nose-up/down angle |
| `roll` | ° | Rotation around longitudinal axis |
| `temp` | °C | Ambient temperature (BMP280) |
| `pressure` | hPa | Absolute pressure |
| `phase` | 0–6 | Current flight state ID |

---

## Ground Station — HYDRO-1 GS
 
The ground station is a separate React + Vite web application that connects to the flight computer over WebSocket and visualizes all telemetry in real time.
 
![Ground Station Dashboard](media/Prototype%20Gallery/ground_station_v2.png)
 
### Features
 
| Feature | Description |
|---|---|
| **3D Attitude View** | Real-time 3D rocket model that rotates with live pitch, roll, and yaw data |
| **Flight Phase Tracker** | Left panel shows all 7 states (STANDBY → ARMED → IGNITION → ASCENT → APOGEE → DESCENT → RECOVERY), highlighting the active state |
| **Live Telemetry Panel** | Right panel — altitude (m), velocity (m/s), G-force, pitch, roll, yaw, temperature (°C), pressure (hPa) |
| **Session Records** | Tracks max altitude, max velocity, max G, and mission elapsed time (T+) |
| **Altitude & Velocity Charts** | Real-time graphs at the bottom of the screen |
| **WiFi Connect / Demo Mode** | Connect to the ESP32 AP or run a simulated flight for testing without hardware |
| **Status Bar** | Persistent bottom bar showing pitch, roll, temperature, and pressure at a glance |
 
### Tech Stack
 
```
React + Vite        — UI framework (localhost:5173 in dev)
WebSocket (port 81) — Real-time data stream from ESP32
Three.js            — 3D rocket attitude visualization
```
 
### Connection Flow
 
```
ESP32 (AP: GANNET) ──► WebSocket ws://192.168.4.1:81 ──► Ground Station
                              10 Hz JSON packets
```

 ![Ground Station Demo](media/Prototype%20Gallery/ground_station_demo.gif)
> **Note:** Ground station source code is maintained in a separate repository. Link coming soon.
 
---

## Repository Structure

```
HydroRocket-Telemetry-System/
├── firmware/
│   └── HydroRocket.ino          # Main flight computer firmware (v2.0)
├── hardware/
│   └── Schematic_HidroRocket_v0_2026-04-20.pdf
├── mechanical/
│   ├── AvionicsBay_v1 - Bay_Structure.stl
│   ├── AvionicsBay_v1 - Sled.stl
│   └── AvionicsBay_v1.step
├── media/
│   └── Prototype Gallery/       # Photos and renders
└── README.md
```

---

## Build & Flash

### Dependencies (Arduino IDE / PlatformIO)

```
MPU9250_WE          by Wolfgang Ewald
Adafruit BMP280     by Adafruit
ESP32Servo          by Kevin Harrington
WebSockets          by Markus Sattler
WiFi                (ESP32 built-in)
```

### Flash Steps

1. Open `firmware/HydroRocket.ino` in Arduino IDE
2. Select board: **ESP32 Dev Module**
3. Select the correct COM port
4. Upload
5. Open Serial Monitor at **115200 baud**
6. Wait for `=== READY FOR FLIGHT ===`
7. Connect to Wi-Fi: `SSID: GANNET` / `Password: 1234`
8. Open ground station on port 81

### Pre-Launch Checklist

- [ ] Battery charged (LiPo 103040, 3.7V nominal)
- [ ] Rocket stationary on pad during 3-second pressure calibration
- [ ] Serial monitor confirms `Base Pressure: ~1013 hPa` (±15 hPa acceptable)
- [ ] Ground station WebSocket connected and receiving data
- [ ] Servo verified at 0° (parachute closed)
- [ ] `phase: 0` (STANDBY) confirmed on dashboard

---

## Known Issues & v4 Integration Status

### Mechanical — v4 Targets

| Item | Status |
|---|---|
| OD validation against physical bottle | ⚠️ Pending — measure with calipers post-print |
| PCB mounting clearance | ⚠️ TBD — single piece, verify nut interference |
| LiPo 103040 compartment fit | ⚠️ TBD — 40×30×10mm + 8mm cable egress |
| Support removal on truss geometry | ⚠️ Risk — 52.03g total with supports vs 23.92g model |

### Firmware — Planned

- [ ] Kalman filter for pitch/roll (replace raw atan2)
- [ ] LittleFS black-box data logging (close file on LANDING state)
- [ ] `ARMED` / `IGNITION` state integration with React ground station
- [ ] In-flight EMI measurement logging (crosstalk baseline vs. shielded config)

### Research — Pending

- [ ] Simscape MATLAB model: mutual inductance between power and signal traces
- [ ] Experimental comparison: unshielded vs. twisted-pair wiring harness

---

## Author

**Jesús Alberto Perea García**
Mechatronics Engineering Student — IEST Anáhuac, Tamaulipas
Member: IEEE Student Branch · Vértice Excellence Program
[github.com/DonJechu](https://github.com/DonJechu) · jesus.perea@iest.edu.mx

*This project is part of ongoing research on EMI in aerospace avionics systems under field-theoretic analysis (Poynting Vector framework). Mentored by Ing. Oscar (aerospace specialist).*
