# four_sensor_table_arc_latched.py
from machine import Pin, PWM
from time import sleep_ms, ticks_ms, ticks_diff

# ----- SENSORS (left->right) -----
S_FL = Pin(20, Pin.IN)   # front-left  (outer)
S_BL = Pin(21, Pin.IN)   # back-left   (inner)
S_BR = Pin(18, Pin.IN)   # back-right  (inner)
S_FR = Pin(19, Pin.IN)   # front-right (outer)
WHITE_LEVEL = 1
def W(x): return x == WHITE_LEVEL

branch_route = ['R','L','S','S','S','S','S','S','S','L','S','L','S','S','S','S','S','S','S','L','S','R']
branch_index = 0

# ----- MOTORS -----
INVERT_LEFT  = False
INVERT_RIGHT = False

class Motor:
    def __init__(self, d, p, inv=False):
        self.dir = Pin(d, Pin.OUT)
        self.pwm = PWM(Pin(p)); self.pwm.freq(1000)
        self.inv = inv
    def fwd(self, sp):
        sp = 0 if sp < 0 else (100 if sp > 100 else int(sp))
        self.dir.value(0 ^ self.inv)
        self.pwm.duty_u16(int(65535*sp/100))
    def stop(self): self.pwm.duty_u16(0)

mL = Motor(4,5, INVERT_LEFT)
mR = Motor(7,6, INVERT_RIGHT)
def go(vL, vR): mL.fwd(vL); mR.fwd(vR)

# ----- TUNING -----
BASE  = 55
DELTA = 20
HARD  = 40
TURN_OUT = 100        # arc outer speed
TURN_IN  = 0         # arc inner speed
DEBOUNCE = 1          # front trigger persistence       # centered cycles to finish turn
MIN_TURN_MS   = 400   # mandatory minimum turn time
MAX_TURN_MS   = 1500  # safety cap
STABLE        = 5     # consecutive centered readings to end the turn
SETTLE_MS     = 120   # short straight run after finishing

DT_MS = 20
UNLATCH_CLEAR_N = 3   # non-branch cycles to unlatch

def arc(side):
    # NO index increment here
    if side == 'R': go(TURN_IN, TURN_OUT)
    else:           go(TURN_OUT, TURN_IN)

def centered(c):     return c == 0b0110
def slight_left(c):  return c == 0b0100
def slight_right(c): return c == 0b0010

def read_code():
    b3 = 1 if W(S_FL.value()) else 0
    b2 = 1 if W(S_BL.value()) else 0
    b1 = 1 if W(S_BR.value()) else 0
    b0 = 1 if W(S_FR.value()) else 0
    return (b3<<3)|(b2<<2)|(b1<<1)|b0

def is_branch_pattern(c):
    # 1111 = junction; 1110 = left branch; 0111 = right branch
    return c in (0b1111, 0b1110, 0b0111)

def main():
    global branch_index
    turning = None
    fl_cnt = fr_cnt = 0
    stable = 0
    t0 = 0

    branch_latched = False
    clear_cnt = 0

    while True:
        c = read_code()
        print("Code:", bin(c))
        FL = (c>>3)&1;  FR = c&1
        mid = (c>>1)&0b11

        # ---------- unlatch when we are clearly off a branch pattern ----------
        if not is_branch_pattern(c):
            clear_cnt += 1
            if clear_cnt >= UNLATCH_CLEAR_N:
                branch_latched = False
                clear_cnt = 0
        else:
            clear_cnt = 0

                # ---------- turning state ----------
        if turning:
            elapsed = ticks_diff(ticks_ms(), t0)

            # keep arcing each control tick
            arc(turning)

            # 1) mandatory dwell: don't allow early exit yet
            if elapsed < MIN_TURN_MS:
                sleep_ms(DT_MS)
                continue

            # 2) while we still see a junction pattern, don't finish
            if is_branch_pattern(c):
                stable = 0
                sleep_ms(DT_MS)
                continue

            # 3) finish only when truly centered (0110) and stable
            if centered(c):
                stable += 1
            else:
                stable = 0

            if stable >= STABLE or elapsed >= MAX_TURN_MS:
                turning = None
                stable = 0
                go(BASE, BASE)        # post-turn settle
                sleep_ms(SETTLE_MS)
                continue

            sleep_ms(DT_MS)
            print("Turning...");
            continue

            if centered(c) or slight_left(c) or slight_right(c):
                stable += 1
            else:
                stable = 0
            if stable >= STABLE or ticks_diff(ticks_ms(), t0) > MAX_TURN_MS:
                turning = None; stable = 0; go(BASE, BASE); sleep_ms(DT_MS); continue
            arc(turning); sleep_ms(DT_MS); continue

        # ---------- branch decision (consume exactly once) ----------
        if is_branch_pattern(c) and not branch_latched:
            # Decide which side the geometry suggests
            geom = 'L' if c == 0b1110 else ('R' if c == 0b0111 else 'X')  # X=1111 (junction)
            action = branch_route[branch_index] if branch_index < len(branch_route) else 'S'

            # Debounce side sensors for 1110/0111; 1111 follows action directly
            if c == 0b1110 and mid == 0b11:
                fl_cnt += 1; fr_cnt = 0
                if fl_cnt < DEBOUNCE: sleep_ms(DT_MS); continue
            elif c == 0b0111 and mid == 0b11:
                fr_cnt += 1; fl_cnt = 0
                if fr_cnt < DEBOUNCE: sleep_ms(DT_MS); continue
            else:
                fl_cnt = fr_cnt = 0

            if action == 'S':
                branch_index += 1          # consume skip ONCE
                branch_latched = True
                mL.stop(); mR.stop()
                sleep_ms(DT_MS); continue

            # Pick turn direction: for 1111 use route action, else trust geometry but
            # still require it to match the route action
            side = action if geom == 'X' else geom
            if action in ('L','R') and (geom == 'X' or action == geom):
                turning = side
                t0 = ticks_ms()
                branch_index += 1          # consume exactly once here
                branch_latched = True
                # no blocking sleep; turning loop handles motion
                continue
            else:
                # geometry doesn't match route action -> ignore and wait
                branch_latched = True      # still consume? choose NOT to consume
                sleep_ms(DT_MS); continue

        # ---------- follower toward 0110 ----------
        if centered(c):
            go(BASE, BASE)
        elif c == 0b0100:
            go(BASE-DELTA, BASE+DELTA)
        elif c == 0b0010:
            go(BASE+DELTA, BASE-DELTA)
        elif c in (0b1100, 0b1000):
            go(BASE-HARD, BASE+HARD)
        elif c in (0b0011, 0b0001):
            go(BASE+HARD, BASE-HARD)
        else:
            go(BASE, BASE-10)

        sleep_ms(DT_MS)

main()
