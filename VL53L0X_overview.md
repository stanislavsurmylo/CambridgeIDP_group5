## VL53L0X ToF Distance Reader (I2C)

###
- VL53L0X Time-of-Flight sensor (30–2000 mm)
- Smart sensor with built-in laser timing and processing
- Communicates digitally over I2C (needs clock + data wires)

### Wiring
- `SDA` ➜ GP8
- `SCL` ➜ GP9
- `VCC` ➜ 3V3 (works 3.3–5 V)
- `GND` ➜ GND

### Notes
- I2C allows multiple devices on the same bus (address 0x29 default)
- Readings are 1 mm resolution, accuracy about ±3%
- Keep sensor still during init; avoid covering the aperture


