# sen0017_test.py — digital D0 on GP26
from machine import Pin
import time

SENSOR_PIN = 26           # GP26 (pin 31)
led = Pin("LED", Pin.OUT) # on-board LED
sensor = Pin(SENSOR_PIN, Pin.IN)

last = None
while True:
    d = sensor.value()           # 1 or 0 from the module
    led.value(d)                 # light when signal high
    if d != last:
        print("white" if d else "black")
        last = d
    time.sleep_ms(20)
