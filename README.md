# 🚀 HYDRO-1 Flight Computer v2.0

**HYDRO-1** is a high-performance avionics system designed for experimental water-powered rockets. It features real-time telemetry, a multi-stage flight state machine, and triple-redundant parachute deployment logic.

This version (v2.0) is "Flight-Ready," with filters and thresholds specifically calibrated to handle the high-vibration environment and aerodynamic turbulence of a real launch.

## 🛰️ System Architecture

### Hardware Stack
* **Core:** ESP32 (Dual-core, WiFi-enabled).
* **IMU:** MPU9250 (9-axis Accelerometer/Gyroscope/Magnetometer) — *Configured for high-G (16G) maneuvers.*
* **Barometer:** BMP280 (High-precision altitude) — *Hardware IIR filter enabled to eliminate pressure transients.*
* **Actuator:** High-torque Servo for parachute deployment.
* **Telemetry:** WebSockets via dedicated WiFi AP (**SSID: GANNET**).

### Data Pipeline & Filtering
The system uses a combination of **Hardware IIR filters** (inside the BMP280) and **Software IIR filters** (Alpha 0.8) to process altitude and velocity. This ensures the system is responsive to 20 m/s changes while ignoring "noise" from aerodynamic flow.



## 🧠 Flight State Machine
The computer transitions through 7 distinct flight phases to ensure safety and mission success:

1.  **STANDBY:** System calibration and waiting for liftoff (> 2.5G confirmed).
2.  **ASCENT:** Real-time calculation of altitude, velocity, and orientation.
3.  **APOGEE:** Triple-redundant detection (Velocity crossing zero, Altitude drop, or Safety Timeout).
4.  **DESCENT:** Parachute deployment and controlled fall.
5.  **LANDING:** Touchdown detection and flight data preservation.

## 🛡️ Safety & Redundancy
To prevent "early deployment" (the most common failure in rocketry), the computer uses three simultaneous methods to confirm the peak of flight (Apogee):
* **Method A (Primary):** Velocity zero-crossing.
* **Method B (Backup):** Altitude drop threshold (1.0m drop).
* **Method C (Fail-safe):** Hard-coded timer (12 seconds) to force deployment.

## 📡 Real-time Telemetry
The system broadcasts a JSON payload at **10Hz** via WebSockets for Ground Station visualization:
```json
{
  "alt": 45.2,
  "vel": 12.5,
  "accel": 3.12,
  "pitch": 85.0,
  "roll": 10.2,
  "temp": 28.5,
  "phase": 3
}
```

## 🛠️ Setup & Configuration
### Dependencies
* `Wire.h`, `WiFi.h`
* `MPU9250_WE`
* `Adafruit_BMP280`
* `ESP32Servo`
* `WebSocketsServer`

### Calibration
Upon boot, the computer performs a **300-sample base pressure calibration** (approx. 3 seconds). Ensure the rocket is stationary on the launch pad during this phase.

---
**Developed by Jesus Perea (DonJechu)** *Mechatronics Engineering Student | Aerospace Enthusiast*
