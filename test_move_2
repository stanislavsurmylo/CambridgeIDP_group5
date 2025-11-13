from machine import Pin, PWM
from time import sleep

SENSOR_PIN_BACK   = 26  # GP26 (pin 31)
SENSOR_PIN_LEFT   = 27  # GP27 (pin 32)
SENSOR_PIN_RIGHT  = 28  # GP28 (pin 34)
SENSOR_PIN_CENTER = 12  # GP12 (pin 15)

led = Pin("LED", Pin.OUT)  # on-board LED

sensor_back   = Pin(SENSOR_PIN_BACK,   Pin.IN)
sensor_left   = Pin(SENSOR_PIN_LEFT,   Pin.IN)
sensor_right  = Pin(SENSOR_PIN_RIGHT,  Pin.IN)
sensor_center = Pin(SENSOR_PIN_CENTER, Pin.IN)

# --- Tunables (adjust to taste) ---
SPEED_FWD      = 100   # % for forward cruising
SPEED_TURN     = 100   # % for spins
SPEED_PIVOT    = 40   # % when pivoting around one wheel
SHIFT_MS       = 120  # small forward shift before spins
CHECK_MS       = 10   # loop wait while checking sensors
#TIMEOUT_MS     = 1500 # max time to search before giving up

def is_line(v):
    return v == 1


class Node:
    def __init__(self, is_start=False, is_end=False):
        self.is_start = is_start
        self.is_end = is_end
        self.adj_nodes = []
        self.is_load_node = False


class Motor:
    def __init__(self, dirPin, PWMPin):
        self.mDir = Pin(dirPin, Pin.OUT)
        self.pwm = PWM(Pin(PWMPin))
        self.pwm.freq(1000)
        self.pwm.duty_u16(0)

    def off(self):
        self.pwm.duty_u16(0)

    def Forward(self, speed):
        self.mDir.value(0)
        self.pwm.duty_u16(int(65535 * speed / 100))

    def Reverse(self, speed):
        self.mDir.value(1)
        self.pwm.duty_u16(int(65535 * speed / 100))

    def Stop(self):
        self.pwm.duty_u16(0)

# Convenience wrappers (assuming motorL = LEFT, motorR = RIGHT).
def go_forward(mL, mR, speed):
    mL.Forward(speed)
    mR.Forward(speed)

def stop_all(mL, mR):
    mL.Stop()
    mR.Stop()

def rotate_left(mL, mR, speed):
    # Left wheel backward, right wheel forward
    mL.Reverse(speed)
    mR.Forward(speed)

def rotate_right(mL, mR, speed):
    # Left wheel forward, right wheel backward
    mL.Forward(speed)
    mR.Reverse(speed)

def pivot_right(mL, mR, speed):
    # Left wheel stationary, right wheel backward => yaw to the RIGHT
    mL.Reverse(speed // 7)
    mR.Reverse(speed)

def pivot_left(mL, mR, speed):
    # Right wheel stationary, left wheel backward => yaw to the LEFT
    mR.Reverse(speed // 7)
    mL.Reverse(speed)

def shift_forward(mL, mR, speed, ms):
    go_forward(mL, mR, speed)
    sleep(ms / 1000)
    stop_all(mL, mR)

def rotation_check():
    while True:
        if is_line(sensor_center.value()):
            return True
        #sleep(CHECK_MS / 1000)

def test_move():
    motorL = Motor(dirPin=4, PWMPin=5)  # LEFT  motor on GP4/GP5
    motorR = Motor(dirPin=7, PWMPin=6)  # RIGHT motor on GP6/GP7

    while True:
        # Read raw sensor values (0=line, 1=not line)
        raw_b = sensor_back.value()
        raw_l = sensor_left.value()
        raw_r = sensor_right.value()
        raw_c = sensor_center.value()

        # Convert to booleans: True when on line
        on_back   = is_line(raw_b)
        on_left   = is_line(raw_l)
        on_right  = is_line(raw_r)
        on_center = is_line(raw_c)

        # --- Core behaviour ---

        # 1) Center & back both see the line ⇒ go straight
        if on_back and on_center:
            go_forward(motorL, motorR, SPEED_FWD)
            #sleep(CHECK_MS / 1000)
            continue

        # 2) Side sees line (priority to RIGHT), do:
        #    (a) short forward shift
        #    (b) spin-in-place toward that side until center sees line again
        if on_right or on_left and on_center:
            # Priority to right if both true
            if on_right:
                shift_forward(motorL, motorR, SPEED_FWD, SHIFT_MS)
                # Spin RIGHT on the spot until center reacquires
                rotate_right(motorL, motorR, SPEED_TURN)
                x = rotation_check()
                stop_all(motorL, motorR)
            else:
                # Left only
                shift_forward(motorL, motorR, SPEED_FWD, SHIFT_MS)
                # Spin LEFT on the spot until center reacquires
                rotate_left(motorL, motorR, SPEED_TURN)
                x = rotation_check()
                stop_all(motorL, motorR)

            # After handling a side event, loop
            #sleep(CHECK_MS / 1000)
            #continue

        # 3) If center LOST the line but a side sensor has it:
        #    “turn around the opposite wheel to get back on the line”
        #    (Right sees line: keep LEFT stationary, drive RIGHT backward → yaw RIGHT)
        if not on_center and (on_right or on_left):
            if on_right:
                pivot_right(motorL, motorR, SPEED_PIVOT)
                rotation_check()
                stop_all(motorL, motorR)
            else:  # on_left
                pivot_left(motorL, motorR, SPEED_PIVOT)
                rotation_check()
                stop_all(motorL, motorR)
            #sleep(CHECK_MS / 1000)
            #continue

        # 4) Fallback: if nothing is detected clearly, stop briefly (or creep forward)
        stop_all(motorL, motorR)
        sleep(CHECK_MS / 1000)

if __name__ == "__main__":
    test_move()
