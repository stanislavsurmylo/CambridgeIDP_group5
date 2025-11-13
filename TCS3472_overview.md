## TCS3472 Color Sensor Reader (I2C)

###
- TCS3472 RGB color and ambient light sensor
- Measures red, green, blue color values and light intensity
- I2C digital interface (3.3–5 V compatible)
- Onboard LED for illumination

### Wiring
- `SDA` ➜ GP4
- `SCL` ➜ GP5
- `VCC` ➜ 3V3 (or 5V)
- `GND` ➜ GND

### Notes
- Returns RGB tuple (red, green, blue) and light value
- I2C address: 0x29 (default)
- Readings update every 1 second (configurable)
- Requires driver: `libs/tcs3472_micropython/tcs3472.py`


