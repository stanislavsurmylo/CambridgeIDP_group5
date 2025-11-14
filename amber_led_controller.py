"""
Amber/Yellow LED Controller with Electrical Isolation
- Uses transistor for isolation (GPIO → Transistor → LED)
- Blinks when AGV is moving or loading/unloading
- OFF when in start area (proves it's being switched)
"""
from machine import Pin
import time

# LED Control Pin (drives transistor base, NOT directly connected to LED)
LED_CONTROL_PIN = 25  # GPIO pin that controls transistor

# State definitions
STATE_START_AREA = "start"
STATE_MOVING = "moving"
STATE_LOADING = "loading"
STATE_UNLOADING = "unloading"

class AmberLEDController:
    """
    Controls amber LED through transistor for electrical isolation.
    
    Circuit:
    GPIO Pin → Resistor (1kΩ) → Transistor Base
    Transistor Collector → LED → Resistor (220Ω) → GND
    Transistor Emitter → GND
    
    This isolates the Pico from the LED circuit.
    """
    
    def __init__(self, control_pin=LED_CONTROL_PIN):
        """
        Initialize LED controller.
        
        Args:
            control_pin: GPIO pin number that drives the transistor
        """
        self.control_pin = Pin(control_pin, Pin.OUT)
        self.control_pin.off()  # Start with LED OFF
        self.current_state = STATE_START_AREA
        self.blink_interval = 0.5  # 0.5 seconds on/off = 1 Hz blink
        
    def set_state(self, state):
        """
        Set AGV state and control LED accordingly.
        
        Args:
            state: One of STATE_START_AREA, STATE_MOVING, STATE_LOADING, STATE_UNLOADING
        """
        self.current_state = state
        
    def update(self):
        """
        Update LED based on current state.
        Call this in your main loop.
        
        - START_AREA: LED OFF (proves switching works)
        - MOVING/LOADING/UNLOADING: LED blinks
        """
        if self.current_state == STATE_START_AREA:
            # LED must be OFF in start area to prove it's being switched
            self.control_pin.off()
        else:
            # Blink LED when moving or loading/unloading
            self.control_pin.toggle()
            
    def blink_once(self):
        """Blink LED once (for testing)"""
        self.control_pin.on()
        time.sleep(self.blink_interval)
        self.control_pin.off()
        time.sleep(self.blink_interval)
        
    def off(self):
        """Turn LED off"""
        self.control_pin.off()
        
    def on(self):
        """Turn LED on (for testing only - use update() in normal operation)"""
        self.control_pin.on()


def is_in_start_area(sensor_back, sensor_center):
    """
    Detect if AGV is in start area.
    Modify this based on your start area detection method.
    
    Example: Start area might have a specific sensor pattern
    """
    # Example: Start area detected when back sensor sees line
    # Adjust this logic based on your actual start area markers
    return sensor_back.value() == 1  # Assuming 1 = line detected


def is_loading_unloading():
    """
    Detect if AGV is loading or unloading.
    Modify this based on your loading/unloading detection.
    
    Could use:
    - Distance sensor (object detected)
    - Color sensor (specific color detected)
    - Time-based (at loading station for X seconds)
    - Manual flag
    """
    # Placeholder - implement based on your sensors
    return False


def main():
    """Test the LED controller"""
    led = AmberLEDController()
    
    print("Testing Amber LED Controller")
    print("1. Testing OFF state (start area)")
    led.set_state(STATE_START_AREA)
    for _ in range(5):
        led.update()
        time.sleep(0.5)
        print(f"  LED should be OFF - State: {led.current_state}")
    
    print("\n2. Testing BLINK state (moving)")
    led.set_state(STATE_MOVING)
    for _ in range(10):
        led.update()
        time.sleep(0.5)
        print(f"  LED blinking - State: {led.current_state}")
    
    print("\n3. Back to OFF (start area)")
    led.set_state(STATE_START_AREA)
    for _ in range(5):
        led.update()
        time.sleep(0.5)
        print(f"  LED should be OFF - State: {led.current_state}")
    
    led.off()
    print("\nTest complete.")


if __name__ == "__main__":
    main()

