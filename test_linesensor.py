# sen0017_test.py  — MicroPython
from machine import Pin
import time

# === CONFIG ===
SENSOR_PIN = 12          # GP16 -> D0 (through level shifting / divider)
LED_PIN = 28          # On-board LED

# === SETUP ===
sensor = Pin(SENSOR_PIN, Pin.IN)
led = Pin(LED_PIN, Pin.OUT)

last_state = None
while True:
    clr = sensor.value()           # 0 for black or 1 for white

    # Light LED when black line is detected
    led.value(clr)

    # Print only on change to keep output tidy
    if clr != last_state:
        if clr == 0:
            print("white")
        else:
            print("black")
        last_state = clr

    time.sleep_ms(20)
