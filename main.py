# four_sensor_table_arc.py — 4-bit follower + debounced arc turns
from machine import Pin, PWM
from time import sleep_ms, ticks_ms, ticks_diff
import map
from map import V
import map
from map import V

# ----- SENSORS (left->right). Swap sFL,sFR wires here if needed -----
S_FL = Pin(19, Pin.IN)   # front-left  (outer)
S_BL = Pin(21, Pin.IN)   # back-left   (inner)
S_BR = Pin(18, Pin.IN)   # back-right  (inner)
S_FR = Pin(20, Pin.IN)   # front-right (outer)
WHITE_LEVEL = 1
def W(x): return x == WHITE_LEVEL


# branch_route = []  path = list of vertices; route = list of 'L','R','F','B'
# branch_index = 0
current_heading = 0
current_vertex = V.START
finish_vertex = None
finish_heading = None


# ----- MOTORS -----
INVERT_LEFT  = False     # flip these until forward() drives robot forward
INVERT_RIGHT = False



class Motor:   
    def __init__(self, d, p, inv=False):
        self.dir=Pin(d,Pin.OUT); self.pwm=PWM(Pin(p)); self.pwm.freq(1000); self.inv=inv
    def fwd(self, sp): self.dir.value(0 ^ self.inv); self.pwm.duty_u16(int(65535*max(0,min(100,int(sp)))/100))
    def bwd(self, sp): self.dir.value(1 ^ self.inv); self.pwm.duty_u16(int(65535*max(0,min(100,int(sp)))/100))
    def stop(self): self.pwm.duty_u16(0)

mL = Motor(4,5, INVERT_LEFT)
mR = Motor(7,6, INVERT_RIGHT)

# ----- TUNING -----
BASE  = 60    # straight speed
DELTA = 20    # small correction to reach 0110
HARD  = 40    # strong correction when far(turns)
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
    # Speed is 15 cm/F at power 60
    #  At power 100, speed is 25 cm/F
    # Time to turn 25.9 cm at 25 cm/F = 1.036 F

def read_code():
    # return 4-bit left->right
    b3 = 1 if W(S_FL.value()) else 0
    b2 = 1 if W(S_BL.value()) else 0
    b1 = 1 if W(S_BR.value()) else 0
    b0 = 1 if W(S_FR.value()) else 0
    return (b3<<3)|(b2<<2)|(b1<<1)|b0


def go(vL, vR): mL.fwd(vL); mR.fwd(vR)
def spin_left(v): mL.bwd(v); mR.fwd(v)
def spin_right(v): mL.fwd(v); mR.bwd(v)

def arc(side):
    global current_heading
    add_branch_index = 0

    if side=='R':
        DEGREE_OF_TURN = 90
        add_branch_index += 1
        go(TURN_OUT, TURN_IN)   # inner slower
        turn_sleep(DEGREE_OF_TURN, (TURN_OUT - TURN_IN))
        print('R')

    elif side=='L':
        DEGREE_OF_TURN = 90     
        add_branch_index += 1
        go(TURN_IN, TURN_OUT)
        turn_sleep(DEGREE_OF_TURN, (TURN_OUT - TURN_IN))
        print('L')
    
    elif side == 'B': 
        DEGREE_OF_TURN = 180
        add_branch_index += 1
        go(TURN_IN, TURN_OUT)
        turn_sleep(DEGREE_OF_TURN, (TURN_OUT - TURN_IN))
        print('B')

    elif side == 'F':
        add_branch_index += 1        # consume the 'F'
        go(100,100)
        sleep_ms(200)  # move forward length of line
        print('F')


    else:
        add_branch_index += 1        # consume the 'F'
        mL.stop(); mR.stop()
        sleep_ms(DT_MS)
        print('X(F)')
    return add_branch_index



def turn_sleep(deg, speed):
    distance = (deg / 180) * 3.14 * RADIUS_OF_TURN  # distance to travel
    time_ms = (distance / (speed * 0.25)) * 1000  # time in ms
    sleep_ms(int(time_ms))

def spin_sleep(deg, speed):
    distance = (deg / 180) * 3.14 * RADIUS_OF_TURN  # distance to travel
    time_ms = (distance*0.65 / (speed * 0.25)) * 1000 * 0.9  # time in ms
    sleep_ms(int(time_ms))


def centered(c):   return c == 0b0110
def slight_left(c):  return c == 0b0100
def slight_right(c): return c == 0b0010

# def skip_loading_bay():
#     complete_route(['F','F','F','F','F','F','F','F'])



def path_to_route(path):
    global finish_heading
    route = []
    prev = path[0]
    curr = path[1]
    for edge in map.DIRECTED_EDGES:
        if edge.src == prev and edge.dst == curr:
            if edge.start_heading - current_heading == 1 or edge.start_heading - current_heading == -3:
                route.append('R')
            elif edge.start_heading - current_heading == -1 or edge.start_heading - current_heading == 3:
                route.append('L')
            elif edge.start_heading - current_heading == 2 or edge.start_heading - current_heading == -2:
                route.append('B')
            elif edge.start_heading - current_heading == 0:
                route.append('F')
            break
    for i in range(0, len(path)):
        if i == 0:
            continue
        prev = path[i-1]
        curr = path[i]
        if i + 1 < len(path):
            next = path[i+1]
        else:
            next = None
        # find the directed edge that matches prev -> curr
        for edge in map.DIRECTED_EDGES:
            if edge.src == prev and edge.dst == curr:
                if edge.src in [V.B_DOWN_BEG, V.B_DOWN_END, V.A_DOWN_BEG, V.A_DOWN_END, V.B_UP_BEG, V.B_UP_END, V.A_UP_BEG, V.A_UP_END] and edge.dst in [V.B_DOWN_BEG, V.B_DOWN_END, V.A_DOWN_BEG, V.A_DOWN_END, V.B_UP_BEG, V.B_UP_END, V.A_UP_BEG, V.A_UP_END]:
                    for i in range(7):
                        route.append('F')
                else:
                    route.append(edge.turn)
                    break
    finish_heading = edge.end_heading  
        
    return route




def complete_route(branch_route):
    global current_heading
    global current_vertex
    branch_index = 0

    if branch_route[branch_index] == 'R':
        spin_right(BASE)
        spin_sleep(90, BASE)
    elif branch_route[branch_index] == 'L':
        spin_left(BASE)
        spin_sleep(90, BASE)
    elif branch_route[branch_index] == 'B':
        spin_right(BASE)
        spin_sleep(180, BASE)
    branch_index = 1

    turning = None
    fl_cnt = fr_cnt = 0
    stable = 0
    t0 = 0
    while branch_index < len(branch_route) or not centered(read_code()):
        print(current_heading)
        c = read_code()
        #print("Code:",bin(c))
        FL = (c>>3)&1;  FR = c&1
        mid = (c>>1)&0b11  # inner pair
                # If all four see white: follow the next route directive
        #print(branch_index)
        
        if c == 0b1111:
            # if we ran out of directives, default to 'F'
            action = branch_route[branch_index] if branch_index < len(branch_route) else 'X'

            if action == 'L':
                turning = 'L'
                t0 = ticks_ms()
                branch_index += arc('L')                 # branch_index += arc() will consume this route entry
                sleep_ms(DT_MS)
                continue

            elif action == 'R':
                turning = 'R'
                t0 = ticks_ms()
                branch_index += arc('R')                 # branch_index += arc() will consume this route entry
                sleep_ms(DT_MS)
                continue

            elif action == 'B':
                turning = 'B'
                t0 = ticks_ms()
                branch_index += arc('B')                 # branch_index += arc() will consume this route entry
                sleep_ms(DT_MS)
                continue

            else:  # 'F' -> skip this node
                turning = 'F'
                t0 = ticks_ms()
                branch_index += arc('F')                 # branch_index += arc() will consume this route entry
                sleep_ms(DT_MS)
                continue
        
        if c == 0b1110:
            # if we ran out of directives, default to 'F'
            action = branch_route[branch_index] if branch_index < len(branch_route) else 'F'

            if action == 'L':
                turning = 'L'
                t0 = ticks_ms()
                branch_index += arc('L')                 # branch_index += arc() will consume this route entry
                sleep_ms(DT_MS)
                continue

            elif action == 'R':
                continue

            else:  # 'F' -> skip this node
                turning = 'F'
                t0 = ticks_ms()
                branch_index += arc('F')                 # branch_index += arc() will consume this route entry
                sleep_ms(DT_MS)
                continue
        
        if c == 0b0111:
            # if we ran out of directives, default to 'F'
            action = branch_route[branch_index] if branch_index < len(branch_route) else 'F'

            if action == 'L':
                continue

            elif action == 'R':
                turning = 'R'
                t0 = ticks_ms()
                branch_index += arc('R')                 # branch_index += arc() will consume this route entry
                sleep_ms(DT_MS)
                continue

            else:  # 'F' -> skip this node
                turning = 'F'
                t0 = ticks_ms()
                branch_index += arc('F')                 # branch_index += arc() will consume this route entry
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
            branch_index += arc(turning); sleep_ms(DT_MS); 
        # completely useless code, delete after testing



        # ---- detect a branch (outer sensor on one side, inner pair mostly white) ----
        # if FL and not FR and mid == 0b11:
        #     fl_cnt += 1; fr_cnt = 0
        #     if fl_cnt >= DEBOUNCE:
        #         turning = 'L'; t0 = ticks_ms(); branch_index += arc('L'); sleep_ms(DT_MS); continue
        # elif FR and not FL and mid == 0b11:
        #     fr_cnt += 1; fl_cnt = 0
        #     if fr_cnt >= DEBOUNCE:
        #         turning = 'R'; t0 = ticks_ms(); arc('R'); sleep_ms(DT_MS); continue
        # else:
        #     fl_cnt = fr_cnt = 0



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
            # all black or unrecognized -> go foward
            go(BASE, BASE)
        

        sleep_ms(DT_MS)
    go(BASE, BASE)
    sleep_ms(200)
    current_heading = finish_heading


def go_to(finish_vertex):
    global current_vertex

    graph = map.GRAPH
    current_path = map.shortest_path(graph, current_vertex, finish_vertex)
    branch_route = path_to_route(current_path)
    complete_route(branch_route)
    current_vertex = finish_vertex

    print("Path:", current_path)
    print("Route:", branch_route)


loading_bays = [V.B_DOWN_BEG, V.B_UP_BEG, V.A_UP_BEG, V.B_DOWN_BEG]  # list of loading bay vertices
boxes_delivered = 0
number_of_bay = 0
last_checked_bay = loading_bays[0]

def color_to_vertex(color: str) -> V:
    try:
        return V[color.upper()]   # uses enum name lookup
    except KeyError:
        raise ValueError(f"Unknown color: {color!r}")


from linear_actuator import unload_robot
from pickup import seek_and_find


def main():
    global current_vertex
    global last_checked_bay
    global boxes_delivered
    global number_of_bay

    while boxes_delivered < 4:

        go_to(last_checked_bay) 
        # we go to last loading bay spot and check if there are any boxes in there. If there are, we pick them up and transport them.
        if seek_and_find(last_checked_bay) is not None: #if we found any boxes there
            color = seek_and_find(last_checked_bay)  # get the color of the box
            delivery_area = color_to_vertex(color)  # map color to vertex
            go_to(delivery_area)  # go to delivery area
            boxes_delivered += 1 # increment boxes delivered
            unload_robot() # unload any boxes we have
        else:
            number_of_bay = (number_of_bay + 1) % len(loading_bays) # set target to next bay


    go_to(V.START)
    go(0,0)
    

main()