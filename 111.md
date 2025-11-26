# Loading Pipeline Overview

## Purpose
- Detect an object near the loader using the TMF8701 range sensor.
- Classify cube color with the TCS3472 only when needed to save power.
- Move the linear actuator to lift the object once distance and color checks pass.

## Hardware
- `TMF8701` time-of-flight sensor on I2C (`I2C_ID=0`, `PIN_SDA=8`, `PIN_SCL=9`).
- `TCS3472` color sensor, power-gated via `COLOR_POWER_PIN=0`.
- `Actuator` driven by direction pin `3` and PWM pin `2`.

## Configuration
- `LOADING_ZONE` selects actuator default positions (1: zone down, 2: zone up).
- Timing constants:
  - `INIT_RETRACT_TIME`: retract to bottom.
  - `ZONE_DOWN_EXTEND_TIME`: default extension for zone down.
  - `LIFT_TIME`: duration of lift once triggered.
- `REFERENCE_DISTANCE_CM` determines trigger distance (default 3 cm).

## Execution Flow
1. **Sensor Setup**  
   `setup_sensor()` initializes TMF8701 and starts distance measurement mode.
2. **Actuator Initialization**  
   `initialize_actuator()` retracts fully, then extends according to `LOADING_ZONE`.
3. **Main Loop** (`main()`):
   - Continuously reads distance via `read_distance_cm()`.
   - When distance ≤ `REFERENCE_DISTANCE_CM`, powers the TCS3472, samples light/RGB, and detects color.
   - If not previously triggered, extends actuator at `ACTUATOR_SPEED` for `LIFT_TIME` seconds, then stops.
   - Resets trigger when distance goes above `REFERENCE_DISTANCE_CM + 1`.

## Color Detection
- `detect_color()` derives normalized RGB ratios.
- Tags `RED`, `GREEN`, `BLUE`, `YELLOW`, or `DARK` / `UNKNOWN` based on thresholds.

## Operation Notes
- Loop delay is `0.2 s`, balancing responsiveness with sensor stability.
- TMF8701 measurement calls use `start()/read()/stop()` for each sample to ensure fresh data.
- Color sensor is only powered when an object is close, reducing noise and power.

## Troubleshooting
- Initialization failures raise explicit `RuntimeError` messages for quick diagnosis.
- Ensure I2C wiring matches configured pins; desktop lint warnings about `machine`/`utime` are expected on non-MicroPython hosts.

