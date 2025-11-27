from machine import Pin
from time import sleep

# Simple button test for Pico (no LED)
# - Button on GPIO 14, wired to GND with internal pull-up
# - Prints a message and count on each detected press

BUTTON_PIN = 14      # your button pin

button = Pin(BUTTON_PIN, Pin.IN, Pin.PULL_UP)
pressed_count = 0


def button_irq(pin):
    """
    IRQ handler for button presses.
    Triggered on falling edge (HIGH -> LOW) when button is pressed.
    """
    global pressed_count

    if not pin.value():  # 0 when pressed
        pressed_count += 1
        print("Button pressed! Count =", pressed_count)


# Trigger on falling edge (button pressed to GND when using pull-up)
button.irq(trigger=Pin.IRQ_FALLING, handler=button_irq)

print("Button test running. Press the button on pin", BUTTON_PIN)

try:
    while True:
        # Main loop just keeps the script alive so IRQs can fire
        sleep(0.1)
except KeyboardInterrupt:
    print("Stopped button test.")


