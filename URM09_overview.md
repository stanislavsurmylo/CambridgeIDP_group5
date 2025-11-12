## URM09 Analog Distance Tracker 

###
- URM09 Ultrasonic Distance Sensor (2–500 cm)
- Outputs an **analog voltage** that changes with distance
-  `distance_cm = (Vout(mV) × 520) / Vin(mV)`
- With 3.3 V supply: `distance_cm ≈ volts × 157.58`

### Wiring
- `AOUT` ➜ GP26 (ADC0)
- `VCC` ➜ 3V3
- `GND` ➜ GND

### Notes
- Analog readings can be noisy—average samples if needed
- NEED calibration!!!

