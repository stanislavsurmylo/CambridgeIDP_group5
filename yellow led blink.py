
from machine import Pin
import rp2
import time

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

def main():
    sm = rp2.StateMachine(0, blink_1hz, freq=2000, set_base=Pin(25))
    sm.active(1)
    
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        sm.active(0)
        # Turn off LED
        led = Pin(25, Pin.OUT)
        led.off()
        print("Stopped.")

if __name__ == "__main__":
    main()

