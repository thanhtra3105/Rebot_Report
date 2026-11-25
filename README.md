# Autonomous Waypoint UAV – Flight Controller Source Code

Course project for **Robotics** – research and development of an autonomous quadcopter that takes off, flies a GPS waypoint mission, and lands on its own.

**Author:** Group 6 - Le Thanh Tra & Tran Dai Tin (22KTMT2)
**Target MCU:** STM32H562 (STM32 HAL, C)

> This package contains only the parts of the firmware discussed in the report (state estimation, flight control, and navigation). Peripheral drivers, the CubeMX project, and the ground-station software are not included, so it is a reference snapshot rather than a standalone build.

## Hardware Overview

| Function | Device |
|---|---|
| IMU (gyro + accelerometer) | ICM-20602 |
| Barometer | DPS310 |
| Magnetometer | IST8310 |
| Optical rangefinder | MTF-01 |
| GNSS | u-blox GPS (UBX protocol) |
| Remote control / telemetry | NRF24 radio link |
| Ground control station | Custom GCS software with 4G|

## Files

| File | Description |
|---|---|
| `kalman.h / kalman.c` | All Kalman filters: **1D**, **3D**, **4D** (see below) |
| `flight_control.h / flight_control.c` | Sensor reading, attitude estimation, cascaded PID, motor mixer, RC input, failsafe |
| `pos_hold.h / pos_hold.c` | Position estimation, position hold, waypoint navigation, auto takeoff/landing, flight-mode state machine |
| `main.c` | Application entry point: peripheral initialization and the periodic task loop that calls the modules above |

## 1. State Estimation – `kalman.c`

| Filter | States | Purpose |
|---|---|---|
| `Kalman1D` | angle | Roll/pitch: gyro rate (predict) + accelerometer angle (update) |
| `Kalman3D` | position, velocity, accel bias | One horizontal axis (North or East): IMU acceleration (predict) + GPS position/velocity (update) |
| `Kalman4D` | altitude, velocity, accel bias, baro bias | Altitude: IMU vertical acceleration (predict) + barometer and rangefinder (updates) |

- The 3D and 4D filters share one **matrix-based core built on CMSIS-DSP** (`arm_mat_*`): `x = Fx + Bu`, `P = FPFᵀ + Q`, and a **Joseph-form** covariance update for numerical stability.
- Measurement noise is **adaptive**: barometer noise grows near the ground (propeller ground effect), and the rangefinder is rejected at high altitude or when its readings jump without matching acceleration (terrain changes).
- GPS noise is taken from the receiver's own accuracy estimates (`hAcc`, `sAcc`).

## 2. Flight Control – `flight_control.c`

- **Attitude estimation (500 Hz):** roll/pitch from `Kalman1D`; yaw from gyro integration, slowly corrected by the tilt-compensated magnetometer heading.
- **Vertical acceleration:** accelerometer rotated into the inertial frame, gravity removed, low-pass filtered, then fed to `Kalman4D`.
- **Cascaded PID control:**
  - Attitude: angle loop → rate loop → motor mixer (X-quad).
  - Altitude: position PID → vertical-velocity PID → throttle correction.
- **Failsafe:** if the radio link is lost for more than 1 s, the vehicle leaves autonomous mode and ramps the throttle down.
- **Logging:** `FLight_Control_Log()` outputs attitude data (accelerometer, gyro-only, complementary filter, and Kalman angles) for filter comparison.

## 3. Position Hold and Navigation – `pos_hold.c`

- **GPS position estimation:** accelerometer data is rotated into the Earth (North/East) frame and used in the `Kalman3D` predict step; GPS fixes drive the update step. Home position is locked only after good GPS quality (≥ 8 satellites, `hAcc` ≤ 2 m, pDOP ≤ 2.5, 10 consecutive samples averaged).
- **Waypoint navigation:** mission waypoints (lat/lon) are converted to local North/East meters relative to home. A **carrot-chasing** scheme moves a virtual target toward the waypoint at up to 2 m/s, slowing down near it; a waypoint is accepted within a 2 m radius, then the next one is loaded.
- **XY controller (`Run_XY_Controller`):** position PID → velocity PID → desired pitch/roll (rotated from Earth to body frame by yaw and limited to ±20°). The same controller is used for position hold, takeoff, landing, and navigation.
- **Optical-flow hold:** an MTF-01 based position-hold path is provided for low-altitude / GPS-denied flight.
- **Flight-mode state machine (`FlightMode_Execute`):** `MANUAL → ARM → TAKEOFF → POSHOLD / ALTHOLD (mission) → LANDING → DISARM`. Takeoff ramps the target altitude and switches to position hold once altitude and velocity are stable; landing descends at a fixed rate and disarms after touchdown.

## Typical Mission Flow

1. Power on, calibrate IMU and barometer, wait for GPS lock and home position.
2. Arm and auto-takeoff to the target altitude.
3. Switch to position hold and follow the uploaded waypoints in order.
4. Auto-land and disarm.
