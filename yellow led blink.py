
from machine import Pin
import rp2
import time

pin_yellow=25

@rp2.asm_pio(set_init=rp2.PIO.OUT_LOW)
def blink_1hz():
    # Cycles: 1 + 7 + 32 * (30 + 1) = 1000
    set(pins, 1)
    set(x, 31)                  [6]
    label("delay_high")
    nop()                       [29]
    jmp(x_dec, "delay_high")

    # Cycles: 1 + 7 + 32 * (30 + 1) = 1000
    set(pins, 0)
    set(x, 31)                  [6]
    label("delay_low")
    nop()                       [29]
    jmp(x_dec, "delay_low")

def is_in_zone():
    """Check if something entered the zone (customize this based on your sensor)"""
    # Example: Use distance sensor, line sensor, or other input
    # For now, placeholder - replace with your actual sensor reading
    return False  # Change this to your sensor logic

def main():
    sm = rp2.StateMachine(0, blink_1hz, freq=2000, set_base=Pin(pin_yellow))
    sm.active(0)  # Start with LED OFF (not in zone)
    
    in_zone = False  # Track previous state
    
    try:
        while True:
            # Check if something entered the zone
            currently_in_zone = is_in_zone()
            
            # If entered zone and wasn't before - activate LED
            if currently_in_zone and not in_zone:
                sm.active(1)  # Start blinking
                print("Entered zone - LED blinking")
                in_zone = True
            
            # If left zone - deactivate LED
            elif not currently_in_zone and in_zone:
                sm.active(0)  # Stop blinking
                led = Pin(pin_yellow, Pin.OUT)
                led.off()  # Make sure LED is off
                print("Left zone - LED off")
                in_zone = False
            
            time.sleep(0.1)  # Check every 100ms
            
    except KeyboardInterrupt:
        sm.active(0)
        led = Pin(pin_yellow, Pin.OUT)
        led.off()
        print("Stopped.")

if __name__ == "__main__":
    main()

