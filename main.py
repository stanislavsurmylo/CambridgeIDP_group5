# four_sensor_table_arc.py — 4-bit follower + debounced arc turns
from machine import Pin, PWM
from time import sleep_ms, ticks_ms, ticks_diff

# ----- SENSORS (left->right). Swap sFL,sFR wires here if needed -----
S_FL = Pin(19, Pin.IN)   # front-left  (outer)
S_BL = Pin(21, Pin.IN)   # back-left   (inner)
S_BR = Pin(18, Pin.IN)   # back-right  (inner)
S_FR = Pin(20, Pin.IN)   # front-right (outer)
WHITE_LEVEL = 1
def W(x): return x == WHITE_LEVEL

branch_route = ['R','L','S','S','S','S','S','S','S','L','S','L','S','S','S','S','S','S','S','L','S','R']  # sequence of turns at branches
branch_index = 0

# ----- MOTORS -----
INVERT_LEFT  = False     # flip these until forward() drives robot forward
INVERT_RIGHT = False

class Motor:
    def __init__(self, d, p, inv=False):
        self.dir=Pin(d,Pin.OUT); self.pwm=PWM(Pin(p)); self.pwm.freq(1000); self.inv=inv
    def fwd(self, sp): self.dir.value(0 ^ self.inv); self.pwm.duty_u16(int(65535*max(0,min(100,int(sp)))/100))
    def stop(self): self.pwm.duty_u16(0)

mL = Motor(4,5, INVERT_LEFT)
mR = Motor(7,6, INVERT_RIGHT)

# ----- TUNING -----
BASE  = 60    # straight speed
DELTA = 20    # small correction to reach 0110
HARD  = 40    # strong correction when far
TURN_OUT = 100 # arc outer wheel speed
TURN_IN  = 0 # arc inner wheel speed
DEBOUNCE = 3  # front trigger must persist N cycles
STABLE   = 1  # need N centered readings to finish a turn
MAX_TURN_MS = 10
DT_MS = 20


RADIUS_OF_TURN = 16.5  # radius of turn in cm

    # Radius of turn = 16.5 cm
    # Angle is 90 degrees
    # turn distance is (π * 16.5) / 2 = 25.9 cm
    # Speed is 15 cm/s at power 60
    #  At power 100, speed is 25 cm/s
    # Time to turn 25.9 cm at 25 cm/s = 1.036 s

def read_code():
    # return 4-bit left->right
    b3 = 1 if W(S_FL.value()) else 0
    b2 = 1 if W(S_BL.value()) else 0
    b1 = 1 if W(S_BR.value()) else 0
    b0 = 1 if W(S_FR.value()) else 0
    return (b3<<3)|(b2<<2)|(b1<<1)|b0


def go(vL, vR): mL.fwd(vL); mR.fwd(vR)

def arc(side):
    global branch_index

    if side=='R':
        DEGREE_OF_TURN = 90
        branch_index += 1
        go(TURN_IN, TURN_OUT)   # inner slower
        turn_sleep(DEGREE_OF_TURN, (TURN_OUT - TURN_IN))
        print('R')

    elif side=='L':
        DEGREE_OF_TURN = 90     
        branch_index += 1
        go(TURN_OUT, TURN_IN)
        turn_sleep(DEGREE_OF_TURN, (TURN_OUT - TURN_IN))
        print('L')
    
    elif side == 'B': 
        DEGREE_OF_TURN = 180
        branch_index += 1
        go(TURN_IN, TURN_OUT)
        turn_sleep(DEGREE_OF_TURN, (TURN_OUT - TURN_IN))
        print('B')

    elif side == 'S':
        branch_index += 1        # consume the 'S'
        go(100,100)
        sleep_ms(200)  # move forward length of line
        print('S')


    else:
        branch_index += 1        # consume the 'S'
        mL.stop(); mR.stop()
        sleep_ms(DT_MS)
        print('X(S)')




def turn_sleep(deg, speed):
    distance = (deg / 180) * 3.14 * RADIUS_OF_TURN  # distance to travel
    time_ms = (distance / (speed * 0.25)) * 1000  # time in ms
    sleep_ms(int(time_ms))


def centered(c):   return c == 0b0110
def slight_left(c):  return c == 0b0100
def slight_right(c): return c == 0b0010

def main():
    global branch_index
    turning = None
    fl_cnt = fr_cnt = 0
    stable = 0
    t0 = 0
    sleep_ms(5000)  # wait for things to settle
    go(BASE, BASE)
    sleep_ms(2000)  # initial settle

    while True:
        c = read_code()
        # print("Code:",bin(c))
        FL = (c>>3)&1;  FR = c&1
        mid = (c>>1)&0b11  # inner pair
                # If all four see white: follow the next route directive
        if c == 0b1111:
            # if we ran out of directives, default to 'S'
            action = branch_route[branch_index] if branch_index < len(branch_route) else 'X'

            if action == 'L':
                turning = 'L'
                t0 = ticks_ms()
                arc('L')                 # arc() will consume this route entry
                sleep_ms(DT_MS)
                continue

            elif action == 'R':
                turning = 'R'
                t0 = ticks_ms()
                arc('R')                 # arc() will consume this route entry
                sleep_ms(DT_MS)
                continue

            elif action == 'B':
                turning = 'B'
                t0 = ticks_ms()
                arc('B')                 # arc() will consume this route entry
                sleep_ms(DT_MS)
                continue

            else:  # 'S' -> skip this node
                turning = 'S'
                t0 = ticks_ms()
                arc('S')                 # arc() will consume this route entry
                sleep_ms(DT_MS)
                continue
        
        if c == 0b1110:
            # if we ran out of directives, default to 'S'
            action = branch_route[branch_index] if branch_index < len(branch_route) else 'S'

            if action == 'L':
                turning = 'L'
                t0 = ticks_ms()
                arc('L')                 # arc() will consume this route entry
                sleep_ms(DT_MS)
                continue

            elif action == 'R':
                continue

            else:  # 'S' -> skip this node
                turning = 'S'
                t0 = ticks_ms()
                arc('S')                 # arc() will consume this route entry
                sleep_ms(DT_MS)
                continue
        
        if c == 0b0111:
            # if we ran out of directives, default to 'S'
            action = branch_route[branch_index] if branch_index < len(branch_route) else 'S'

            if action == 'L':
                continue

            elif action == 'R':
                turning = 'R'
                t0 = ticks_ms()
                arc('R')                 # arc() will consume this route entry
                sleep_ms(DT_MS)
                continue

            else:  # 'S' -> skip this node
                turning = 'S'
                t0 = ticks_ms()
                arc('S')                 # arc() will consume this route entry
                sleep_ms(DT_MS)
                continue

        # ---- turn state ----

        if turning:
            # finish when re-centered or inner pair suggests followable track again
            if centered(c) or slight_left(c) or slight_right(c):
                stable += 1
            else:
                stable = 0
            if stable >= STABLE or ticks_diff(ticks_ms(), t0) > MAX_TURN_MS:
                turning = None; stable = 0; go(BASE, BASE); sleep_ms(DT_MS); continue
            arc(turning); sleep_ms(DT_MS); continue

        # ---- detect a branch (outer sensor on one side, inner pair mostly white) ----
        # if FL and not FR and mid == 0b11:
        #     fl_cnt += 1; fr_cnt = 0
        #     if fl_cnt >= DEBOUNCE:
        #         turning = 'L'; t0 = ticks_ms(); arc('L'); sleep_ms(DT_MS); continue
        # elif FR and not FL and mid == 0b11:
        #     fr_cnt += 1; fl_cnt = 0
        #     if fr_cnt >= DEBOUNCE:
        #         turning = 'R'; t0 = ticks_ms(); arc('R'); sleep_ms(DT_MS); continue
        # else:
        #     fl_cnt = fr_cnt = 0


# ---- follower toward 0110 (no turn) ----
        if centered(c):
            go(BASE, BASE)
        elif c == 0b0100:           # left inner only -> steer LEFT
            go(BASE-DELTA, BASE+DELTA)
        elif c == 0b0010:           # right inner only -> steer RIGHT
            go(BASE+DELTA, BASE-DELTA)
        elif c in (0b1100, 0b1000): # far to left
            go(BASE-HARD, BASE+HARD)
        elif c in (0b0011, 0b0001): # far to right
            go(BASE+HARD, BASE-HARD)
        else:
            # unknown/lost -> gentle bias to move forward
            go(BASE, BASE-10)

        sleep_ms(DT_MS)
        

main()