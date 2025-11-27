from machine import Pin
import rp2
import time

PIN_YELLOW = 17

@rp2.asm_pio(set_init=rp2.PIO.OUT_LOW)
def blink_1hz():
    set(pins, 1)           # LED ON
    set(x, 31)             [6]
    label("delay_high")
    nop()                  [29]
    jmp(x_dec, "delay_high")
    
    set(pins, 0)           # LED OFF
    set(x, 31)             [6]
    label("delay_low")
    nop()                  [29]
    jmp(x_dec, "delay_low")

def is_in_zone():
    """Check if in zone - customize with your sensor logic"""
    # Replace with your actual sensor reading
    return False  # Placeholder


def main():
    sm = rp2.StateMachine(0, blink_1hz, freq=2000, set_base=Pin(PIN_YELLOW))
    in_zone = False
    
    try:
        while True:
            currently_in_zone = is_in_zone()

            if currently_in_zone and not in_zone:
                sm.active(1)
                print("Entered zone - LED blinking")
                in_zone = True
            
            elif not currently_in_zone and in_zone:
                sm.active(0)
                Pin(PIN_YELLOW, Pin.OUT).off()
                print("Left zone - LED off")
                in_zone = False
            
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        sm.active(0)
        Pin(PIN_YELLOW, Pin.OUT).off()
        print("Stopped.")

if __name__ == "__main__":
    main()

