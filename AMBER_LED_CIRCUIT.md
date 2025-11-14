# Amber LED Circuit with Electrical Isolation

## Requirements
- **Electrical Isolation**: LED must NOT be directly connected to Pico GPIO
- **State Control**: LED blinks when moving/loading, OFF in start area
- **Proof of Switching**: LED must be OFF in start area to prove it's being controlled

## Circuit Diagram

```
Pico GPIO Pin 25
    |
    |---[1kΩ Resistor]---|
                         |
                    Transistor Base (NPN, e.g., 2N2222)
                         |
                    Transistor Emitter --- GND
                         |
                    Transistor Collector
                         |
                    [220Ω Resistor]
                         |
                    Amber LED (Anode)
                         |
                    Amber LED (Cathode)
                         |
                    GND
```

## Components Needed
- **NPN Transistor**: 2N2222 or similar (BC547, 2N3904)
- **Base Resistor**: 1kΩ (protects GPIO pin)
- **LED Resistor**: 220Ω (limits LED current)
- **Amber/Yellow LED**: Standard 5mm LED
- **Power Supply**: 3.3V or 5V (separate from Pico if needed)

## Wiring Instructions

1. **GPIO Pin 25** → **1kΩ Resistor** → **Transistor Base**
2. **Transistor Emitter** → **GND** (common ground with Pico)
3. **Power Supply (+) (3.3V or 5V)** → **220Ω Resistor** → **LED Anode**
4. **LED Cathode** → **Transistor Collector**
5. **Power Supply (-)** → **GND** (common with Pico GND)

## How It Works

1. **GPIO HIGH (3.3V)**: 
   - Current flows through base resistor → transistor turns ON
   - Transistor conducts → LED circuit completes → LED lights up

2. **GPIO LOW (0V)**:
   - No base current → transistor turns OFF
   - Transistor blocks current → LED circuit open → LED off

3. **Isolation**:
   - Pico GPIO only drives transistor base (low current, ~1mA)
   - LED current flows through separate power supply
   - Pico is protected from LED circuit faults

## Code Usage

```python
from amber_led_controller import AmberLEDController, STATE_START_AREA, STATE_MOVING

led = AmberLEDController(control_pin=25)

# In start area - LED OFF
led.set_state(STATE_START_AREA)
led.update()  # LED turns OFF

# Moving - LED blinks
led.set_state(STATE_MOVING)
led.update()  # LED toggles (blinks)
```

## Testing

1. **Start Area Test**: 
   - Set state to `STATE_START_AREA`
   - Verify LED is OFF
   - This proves the LED is being switched (not stuck on)

2. **Moving Test**:
   - Set state to `STATE_MOVING`
   - Verify LED blinks at ~1 Hz

3. **Loading Test**:
   - Set state to `STATE_LOADING`
   - Verify LED blinks

## Safety Notes

- ✅ **Isolated**: Pico GPIO never directly powers LED
- ✅ **Protected**: Base resistor limits GPIO current
- ✅ **Flexible**: LED can use different voltage (3.3V or 5V)
- ✅ **Standard Practice**: This is standard engineering practice for output isolation

