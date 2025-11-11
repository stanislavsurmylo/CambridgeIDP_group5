import machine, time
try:
    led = machine.Pin("LED", machine.Pin.OUT)   # Pico W / newer fw
except:
    led = machine.Pin(25, machine.Pin.OUT)      # original Pico fallback

for _ in range(10):
    led.toggle()
    time.sleep(0.2)
