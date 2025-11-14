# two_back_follow.py — MicroPython, 2-sensor line following (no turning logic)
from machine import Pin, PWM
from time import sleep_ms

# === PINS (unchanged) ===
SENSOR_PIN_LEFT   = 21  # GP21 (pin 27)  -> treat as "left-back" (L)
SENSOR_PIN_RIGHT = 18  # GP18 (pin 24)  -> treat as "right-back" (R)

# Optional LED
led = Pin("LED", Pin.OUT)

# Digital sensors
sL = Pin(SENSOR_PIN_LEFT,   Pin.IN)   # left-back
sR = Pin(SENSOR_PIN_RIGHT, Pin.IN)   # right-back

# === TUNING ===
WHITE_LEVEL   = 1        # set to 0 if your module outputs 0 on white
BASE_SPEED    = 35       # forward cruise speed [%]
GAIN          = 25       # differential correction added/subtracted to motors
CHECK_MS      = 20       # loop period

def is_white(v): 
    return v == WHITE_LEVEL

class Motor:
    def __init__(self, dir_pin, pwm_pin, invert=False, freq=1000):
        self.dir = Pin(dir_pin, Pin.OUT)
        self.pwm = PWM(Pin(pwm_pin))
        self.pwm.freq(freq)
        self.invert = invert
        self.stop()

    def stop(self):
        self.pwm.duty_u16(0)

    def forward(self, speed):
        # speed: 0..100 (no reverse in this follower)
        self.dir.value(0 ^ self.invert)
        duty = max(0, min(100, int(speed)))
        self.pwm.duty_u16(int(65535 * duty / 100))

def set_speeds(mL, mR, vL, vR):
    mL.forward(vL); mR.forward(vR)

def follow_loop():
    # LEFT motor on GP4/5, RIGHT motor on GP6/7
    mL = Motor(4, 5, invert=False)
    mR = Motor(7, 6, invert=False)   # flip invert=True/False to match your wiring

    last_err = 0
    while True:
        # Read two back sensors: 1=white (on line), 0=black (off line)
        L = 1 if is_white(sL.value()) else 0
        R = 1 if is_white(sR.value()) else 0
        led.value(L & R)   # LED on when perfectly centered

        # Error definition: +1 = need to steer RIGHT, -1 = steer LEFT
        # (because R more white than L means robot drifted left)
        err = R - L

        if L == 0 and R == 0:
            # Both off-line at the back — search using last direction bias
            # (keep moving, but bias strongly to last_err side)
            bias = GAIN * (1 if last_err >= 0 else -1) * 1.5
            vL = BASE_SPEED + bias
            vR = BASE_SPEED - bias
        else:
            # Differential correction around BASE_SPEED
            bias = GAIN * err
            vL = BASE_SPEED + bias
            vR = BASE_SPEED - bias
            last_err = err

        # Clamp and apply
        vL = max(0, min(100, int(vL)))
        vR = max(0, min(100, int(vR)))
        set_speeds(mL, mR, vL, vR)

        sleep_ms(CHECK_MS)

if __name__ == "__main__":
    follow_loop()
