from machine import Pin, PWM
from time import sleep

SENSOR_PIN_BRIGHT   = 21  # GP21 (pin 27)
SENSOR_PIN_FLEFT   = 20  # GP20 (pin 26)
SENSOR_PIN_FRIGHT  = 19  # GP19 (pin 25)
SENSOR_PIN_BLEFT = 18  # GP18 (pin 24)

led = Pin("LED", Pin.OUT)  # on-board LED

sensor_bright   = Pin(SENSOR_PIN_BRIGHT,   Pin.IN)
sensor_fleft   = Pin(SENSOR_PIN_FLEFT,   Pin.IN)
sensor_fright  = Pin(SENSOR_PIN_FRIGHT,  Pin.IN)
sensor_bleft = Pin(SENSOR_PIN_BLEFT, Pin.IN)


SPEED_FWD      = 100   # % for forward cruising
SPEED_BACK     = 60   # % for reversing when lost
SHIFT_COEF     = 100/210  # proportion of SPEED_FWD for SHIFT_MS
SPEED_TURN     = 80   # % for spins
SPEED_PIVOT    = 40    # % when pivoting around one wheel
SHIFT_MS       = SPEED_FWD / SHIFT_COEF   # small forward shift before spins
CHECK_MS       = 200
DELAY_MS       = 20
# delay for checking sensors
#TIMEOUT_MS     = 1500 # max time to search before giving up

def is_line(v):
    return v == 1

# ----------------- HARD-CODED RED-LINE ROUTE -----------------
# Branch decisions (only when center is on main line):
# 'L' turn left, 'R' turn right, 'S' keep straight (ignore the branch)
BRANCH_ROUTE = ['R','L','S','S','S','S','S','S','S','L','S','L','S','S','S','S','S','S','S','L','S','R']
branch_idx = 0
# -------------------------------------------------------------

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
def go_backward(mL, mR, speed):
    mL.Reverse(speed)
    mR.Reverse(speed)

def go_forward(mL, mR, speed):
    mL.Forward(speed)
    mR.Forward(speed)

def stop_all(mL, mR):
    mL.Stop()
    mR.Stop()

def rotate_left(mL, mR, speed):
    # Left wheel backward, right wheel forward
    mR.Reverse(speed)
    mL.Forward(speed)

def rotate_right(mL, mR, speed):
    # Left wheel forward, right wheel backward
    mR.Forward(speed)
    mL.Reverse(speed)

def rotate_left(mL, mR, speed):
    # Left wheel backward, right wheel forward
    mR.Reverse(speed)
    mL.Forward(speed)

def rotate_right(mL, mR, speed):
    # Left wheel forward, right wheel backward
    mR.Forward(speed)
    mL.Reverse(speed)

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

def left_check():
    sleep(CHECK_MS / 1000)
    while True:
        if is_line(sensor_bleft.value()):
            sleep(DELAY_MS / 1000)
            return True
        #sleep(CHECK_MS / 1000)

def right_check():
    while True:
        if is_line(sensor_bright.value()):
            sleep(DELAY_MS / 1000)
            return True
        #sleep(CHECK_MS / 1000)

def any_check():
    while True:
        if is_line(sensor_bleft.value()) or is_line(sensor_fleft.value()) or is_line(sensor_fright.value()):
            sleep(DELAY_MS / 1000)
            return True
        #sleep(CHECK_MS / 1000)

def line_follow(s1, s2, s3, s4):
    if s2, s3:
        go_forward(motorL, motorR, SPEED_FWD)
    elif s2 and not s3:
        pivot_left(motorL, motorR, SPEED_PIVOT)

    elif not s2 and s3:
        pivot_right(motorL, motorR, SPEED_PIVOT)


def test_move():
    global branch_idx  # <-- routing index
    motorL = Motor(dirPin=4, PWMPin=5)  # LEFT  motor on GP4/GP5
    motorR = Motor(dirPin=7, PWMPin=6)  # RIGHT motor on GP6/GP7

    while True:
        # Read raw sensor values (0=line, 1=not line)
        # Convert to booleans: True when on line
        on_bright  = is_line(sensor_bright.value())
        on_fleft   = is_line(sensor_fleft.value())
        on_fright  = is_line(sensor_fright.value())
        on_bleft   = is_line(sensor_bleft.value())
        
        print(f"B:{on_bright} L:{on_fleft} C:{on_bleft} R:{on_fright}")

        # 2) Side sees line (priority to RIGHT) BUT now gated by the route:
        #    (a) short forward shift
        #    (b) turn as prescribed by BRANCH_ROUTE; otherwise ignore (go straight)
        if (on_fright or on_fleft) and on_bleft and on_bright:
            #Decide what to do at this branch
            if branch_idx < len(BRANCH_ROUTE):
                action = BRANCH_ROUTE[branch_idx]
                branch_idx += 1  # consume this branch event

            if action == 'S':
                # ignore this spur, just clear the node
                shift_forward(motorL, motorR, SPEED_FWD, SHIFT_MS//5)
                continue
            elif action == 'R' and on_fright:
                shift_forward(motorL, motorR, SPEED_FWD, SHIFT_MS)
                rotate_right(motorL, motorR, SPEED_TURN)
                center_check()
                stop_all(motorL, motorR)
                continue
            elif action == 'L' and on_fleft:
                shift_forward(motorL, motorR, SPEED_FWD, SHIFT_MS)
                rotate_left(motorL, motorR, SPEED_TURN)
                center_check()
                stop_all(motorL, motorR)
                continue
            else:
                # Desired turn not available in this instant; ignore and keep moving
                shift_forward(motorL, motorR, SPEED_FWD, SHIFT_MS//5)
                continue
            # if on_fright:
            #     shift_forward(motorL, motorR, SPEED_FWD, SHIFT_MS)
            #     rotate_right(motorL, motorR, SPEED_TURN)
            #     center_check()
            #     stop_all(motorL, motorR)
            #     continue
            # elif on_fleft:
            #     shift_forward(motorL, motorR, SPEED_FWD, SHIFT_MS)
            #     rotate_left(motorL, motorR, SPEED_TURN)
            #     center_check()
            #     stop_all(motorL, motorR)
            #     continue

        # 3) If center LOST the line but a side sensor has it:
        #    corners (outer loop) handled as before
        if not on_bleft and (on_fright or on_fleft) and on_bright:
            if on_fright:
                rotate_right(motorL, motorR, SPEED_TURN)
                center_check()
                stop_all(motorL, motorR)
            else:  # on_fleft
                rotate_left(motorL, motorR, SPEED_TURN)
                center_check()
                stop_all(motorL, motorR)
            #sleep(CHECK_MS / 1000)
            continue

if __name__ == "__main__":
    test_move()


