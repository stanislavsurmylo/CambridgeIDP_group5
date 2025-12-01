# four_sensor_table_arc.py — 4-bit follower + debounced arc turns
import loading_pipeline_state_machine
from machine import Pin, PWM, I2C
from time import sleep_ms, ticks_ms, ticks_diff, sleep
import rp2
import map
from map import V
import vl53l0x_distance
from vl53l0x_distance import setup_sensor_vl53l0x, vl53l0x_read_distance
from libs.tmf8701 import DFRobot_TMF8701
from linear_actuator import Actuator
import loading_pipeline
from loading_pipeline import loading_pipeline_main
from loading_pipeline_state_machine import LoadingPipelineState, pipeline_step

PIN_YELLOW = 17
BUTTON_PIN = 14 

LOADING_ZONE = 1  # zone down = 1, zone up = 2

# I2C pins shared with TMF8701
I2C_ID = 0
PIN_SDA = 8
PIN_SCL = 9

# I2C pins for TCS3472
I2C_ID_TCS3472 = 1
PIN_SDA_TCS3472 = 10
PIN_SCL_TCS3472 = 11

# TCS3472 power control
COLOR_POWER_PIN = 0
COLOR_POWER_STABILIZE_MS = 200
COLOR_INTEGRATION_MS = 500

# Actuator pins
ACTUATOR_DIR_PIN = 3
ACTUATOR_PWM_PIN = 2

# Timing constants
INIT_RETRACT_TIME = 6.0  # Retract to bottommost position
ZONE_DOWN_EXTEND_TIME = 3.0  # Extend to default position for zone_down
ZONE_UP_EXTEND_TIME = 0  # Extend to default position for zone_up
LIFT_TIME = 3.0  # Base lift time when starting loading
ACTUATOR_SPEED = 50

MIN_INIT_DISTANCE_CM = 5
ZONE_PRESET_DISTANCE_CM = 10.0 # Trigger zone preset extend
COLOR_REFERENCE_DISTANCE_CM = 5  # Trigger color sampling
LIFT_REFERENCE_DISTANCE_CM = 3  # Trigger lift phase
LOOP_DELAY = 0.2


# ----- TUNING -----
BASE  = 50    # straight speed
SPIN_BASE = 50
LOAD_BASE = 40
SEEK_COEFF = 0.5  # how aggressively to seek line
DELTA = 17    # small correction to reach 0110
HARD  = 30    # strong correction when far(turns)
TURN_OUT = 100 # arc outer wheel speed
TURN_IN  = 0 # arc inner wheel speed
DEBOUNCE = 3  # front trigger must persist N cycles
STABLE   = 1  # need N centered readings to finish a turn
MAX_TURN_MS = 10                                                 
TARGET_DISTANCE = 250  # target distance in mm   
COLOUR_DETECTION_DISTANCE = 50  # distance to detect color in mm
PICKUP_DISTANCE = 40.0  # distance to pick up box in mm

DT_MS = 10


RADIUS_OF_TURN = 16.5  # radius of turn in cm

    # Radius of turn = 16.5 cm
    # Angle is 90 degrees
    # turn distance is (π * 16.5) / 2 = 25.9 cm
    # Speed is 15 cm/F at power 60
    #  At power 100, speed is 25 cm/F
    # Time to turn 25.9 cm at 25 cm/F = 1.036 F



@rp2.asm_pio(set_init=rp2.PIO.OUT_LOW)
def blink_1hz():
    set(pins, 1)           # LED ON
    set(x, 31)             [6]
    label("delay_high")
    nop()                  [29]
    jmp(x_dec, "delay_high")

    set(pins, 0)           # LED OFF
    set(x, 31)             [6]
    label("delay_low")
    nop()                  [29]
    jmp(x_dec, "delay_low")

# def unload_robot():
#     go(0,0)
#     # Unloading robot to reduce weight on actuator
#     actuator = Actuator(ACTUATOR_DIR_PIN, ACTUATOR_PWM_PIN)
    
#     # Test retract
#     actuator.retract(speed=100)
#     sleep(1)
#     actuator.stop()
#     sleep(1)  # Pause so you can measure
#     print("Unloading complete")

# Global flags for button control
robot_started = False  # True after first button press (start)
emergency_stop = False  # True when paused (after robot has started)
global_actuator = None  # Global actuator reference for emergency stop

def _button_irq(pin):
    global robot_started, emergency_stop, global_actuator
    if not robot_started:
        robot_started = True
        emergency_stop = False
    else:
        emergency_stop = not emergency_stop
        if emergency_stop:
            mL.stop()
            mR.stop()
            # Stop actuator if it exists
            if global_actuator is not None:
                try:
                    global_actuator.stop()
                except:
                    pass

# Trigger on falling edge (button pressed to GND when using pull-up)
button = Pin(BUTTON_PIN, Pin.IN, Pin.PULL_UP)
button.irq(trigger=Pin.IRQ_FALLING, handler=_button_irq)

#vl53l0x distance sensor:
# setup_sensor1 = setup_sensor_vl53l0x()

# colour sensor:
color_power = Pin(COLOR_POWER_PIN, Pin.OUT, value=0)

# Global variable to store initialized sensor (singleton pattern)
_setup_sensor2 = None

def setup_sensor_tmf8701():
    global _setup_sensor2
    # Return existing sensor if already initialized
    if _setup_sensor2 is not None:
        return _setup_sensor2
    
    try:
        i2c = I2C(I2C_ID, sda=Pin(PIN_SDA), scl=Pin(PIN_SCL))
        sensor = DFRobot_TMF8701(i2c)
        if sensor.begin() != 0:
            raise RuntimeError("TMF8701 initialization failed")
        if not sensor.start_measurement(sensor.eMODE_NO_CALIB, sensor.eCOMBINE):
            raise RuntimeError("TMF8701 failed to start measurement")
        print("TMF8701 distance reader ready.")
        _setup_sensor2 = sensor  # Store for reuse
        return sensor
    except OSError as e:
        print(f"I2C error in setup_sensor_tmf8701: {e}")
        raise
    except Exception as e:
        print(f"Error in setup_sensor_tmf8701: {e}")
        raise

def read_distance_mm(sensor):
    try:
        if sensor.is_data_ready():
            dist_mm = sensor.get_distance_mm()
            return dist_mm
    except Exception as e:
        # Handle I2C timeout or other communication errors
        print(f"I2C error in read_distance_mm: {e}")
        return None
    return None

# Global variables for sensor and loading state (initialized in main())
setup_sensor2 = None
loading_state = None


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


def read_code():
    # return 4-bit left->right
    b3 = 1 if W(S_FL.value()) else 0
    b2 = 1 if W(S_BL.value()) else 0
    b1 = 1 if W(S_BR.value()) else 0
    b0 = 1 if W(S_FR.value()) else 0
    return (b3<<3)|(b2<<2)|(b1<<1)|b0


def go(vL, vR): mL.fwd(vL); mR.fwd(vR)
def spin_left(): mL.bwd(SPIN_BASE); mR.fwd(SPIN_BASE)
def spin_right(): mL.fwd(SPIN_BASE); mR.bwd(SPIN_BASE)
def go_back(vL, vR): mL.bwd(vL); mR.bwd(vR)

def shift_with_correction(duration_ms):
    global emergency_stop
    t0 = ticks_ms()
    while ticks_diff(ticks_ms(), t0) < duration_ms:
        if emergency_stop:
            go(0, 0)
            break
        c = read_code()

        if centered(c):
            go(BASE, BASE)
        elif slight_left(c):          # call the function
            go(BASE - DELTA, BASE + DELTA)
        elif slight_right(c):         # call the function
            go(BASE + DELTA, BASE - DELTA)
        elif c in (0b1100, 0b1000):   # far to left
            go(BASE - HARD, BASE + HARD)
        elif c in (0b0011, 0b0001):   # far to right
            go(BASE + HARD, BASE - HARD)
        else:
            # unknown/lost -> move forward
            go(BASE, BASE)

        sleep_ms(DT_MS)

def shift_back_without_correction(duration_ms):
    global emergency_stop
    t0 = ticks_ms()
    while ticks_diff(ticks_ms(), t0) < duration_ms:
        if emergency_stop:
            go(0, 0)
            break
        go_back(BASE, BASE)
        # c = read_code()

        # if centered(c):
        #     go_back(BASE, BASE)
        # elif slight_left(c):          # call the function
        #     go_back(BASE - DELTA, BASE + DELTA)
        # elif slight_right(c):         # call the function
        #     go_back(BASE + DELTA, BASE - DELTA)
        # elif c in (0b1100, 0b1000):   # far to left
        #     go_back(BASE - HARD, BASE + HARD)
        # elif c in (0b0011, 0b0001):   # far to right
        #     go_back(BASE + HARD, BASE - HARD)

        sleep_ms(DT_MS)



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
        go(BASE+10,BASE+10)
        sleep_ms(200)  # move forward length of line
        print('F')

    else:
        add_branch_index += 1        # consume the 'F'
        mL.stop(); mR.stop()
        sleep_ms(DT_MS)
        print('X(F)')
    return add_branch_index

def go_spin_left(deg):
    shift_with_correction((950//BASE)*40)  # move forward length of line
    spin_left()   # inner slower
    spin_sleep(deg)



def turn_sleep(deg, speed):
    distance = (deg / 180) * 3.14 * RADIUS_OF_TURN  # distance to travel
    time_ms = (distance / (speed * 0.25)) * 1000 # time in ms
    sleep_ms(int(time_ms))

def spin_sleep(deg):
    distance = (deg / 180) * 3.14 * RADIUS_OF_TURN  # distance to travel
    time_ms = (distance*0.70 / (SPIN_BASE * 0.25)) * 1000 * 0.9  # time in ms
    sleep_ms(int(time_ms))


def centered(c):   return c == 0b0110
def slight_left(c):  return c == 0b0100
def slight_right(c): return c == 0b0010
def spin_back(deg):
    if current_vertex in [V.B_DOWN_BEG, V.B_DOWN_END, V.A_UP_BEG, V.A_UP_END]:
        spin_left()
        spin_sleep(180)
    else:
        spin_right()
        spin_sleep(180)


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
                if next is None:
                    edge0 = edge
                if edge.src in [V.B_DOWN_BEG, V.B_DOWN_END, V.A_DOWN_BEG, V.A_DOWN_END] and edge.dst in [V.B_DOWN_BEG, V.B_DOWN_END, V.A_DOWN_BEG, V.A_DOWN_END]:
                    for i in range(7):
                        route.append('F')
                elif edge.src in [V.B_UP_BEG, V.B_UP_END, V.A_UP_BEG, V.A_UP_END] and edge.dst in [V.B_UP_BEG, V.B_UP_END, V.A_UP_BEG, V.A_UP_END]:
                    for i in range(6):
                        route.append('F')
                else:
                    route.append(edge.turn)
                    break
    print(edge.src, edge.dst, edge.start_heading, edge.end_heading)
    finish_heading = edge0.end_heading  
        
    return route





def seek_and_find(LoadingBay):
    global current_heading
    global current_vertex
    global emergency_stop
    global loading_state
    global zone
    global number_of_bay
    loading_stage = 0
    turn_counter_on = True
    turn_counter = 0
    colour = None  # Initialize colour variable
    if zone == 'down':
        max_number_of_turns = 7
    else:
        max_number_of_turns = 6
    for edge in map.DIRECTED_EDGES:
        if edge.src == LoadingBay and edge.dst in [V.B_DOWN_END, V.A_DOWN_END, V.B_UP_END, V.A_UP_END]:
            if edge.start_heading - current_heading == 2 or edge.start_heading - current_heading == -2:
                spin_back()
    if LoadingBay == V.B_UP_BEG:
        shift_back_without_correction((950//BASE)*40)
    while turn_counter < max_number_of_turns and not emergency_stop:
        c = read_code()
        print("loading stage:",loading_stage, "turn_counter:", turn_counter)
        FL = (c>>3)&1;  FR = c&1
        mid = (c>>1)&0b11  # inner pair
                # If all four see white: follow the next route directive
        #print(branch_index)
        
        if (c == 0b1110 or c == 0b1111) and loading_stage == 0:
            if turn_counter_on:
                turn_counter += 1
            turn_counter_on = False
            # sensor_distance1 = vl53l0x_read_distance(setup_sensor1)
            # print("Distance:", sensor_distance1)
            # if sensor_distance1 < TARGET_DISTANCE:  # long-range distance sensor needs fixing
            if True:
                # tick1 = ticks_ms()
                # if tick1 - tick0 > 50: #???(debounce?)
                loading_stage = 1
                continue
            
                
        elif loading_stage == 0:  
            turn_counter_on = True

        # if c == 0b1110 and box_found:
        #     x += arc('L')                 # branch_index += arc() will consume this route entry
        #     sleep_ms(DT_MS)
        #     continue

        if c == 0b1110 and loading_stage == 1:
            if current_vertex == V.B_DOWN_BEG and turn_counter == max_number_of_turns - 1:
                number_of_bay = (number_of_bay + 1) % len(loading_bays)
            elif turn_counter == max_number_of_turns:
                number_of_bay = (number_of_bay + 1) % len(loading_bays)
            go_spin_left(90)                 # branch_index += arc() will consume this route entry
            sleep_ms(DT_MS)
            loading_stage = 2
            tick0 = ticks_ms()
            continue

        elif loading_stage == 2:
            sensor_distance2 = read_distance_mm(setup_sensor2)
            print("Distance:", sensor_distance2)
            delta_tick = ticks_diff(ticks_ms(), tick0)
            
            # Check if object is within pickup distance
            if sensor_distance2 != None:
                if sensor_distance2 < PICKUP_DISTANCE and sensor_distance2 > 0:
                    print("Object detected within pickup distance, moving to loading_stage 3")
                    loading_stage = 3
                    continue
            
            # Timeout handling: if no object detected within 2 seconds, reset
            if delta_tick > 3000:  # timeout after 2 seconds
                print("Timeout: No object detected within 2 seconds, resetting to stage 0")
                shift_back_without_correction((16//5.5)*((950//BASE)*40))
                spin_right()
                spin_sleep(90)
                loading_stage = 0
                if current_vertex == V.B_DOWN_BEG and turn_counter == max_number_of_turns - 1:
                    number_of_bay = (number_of_bay - 1) % len(loading_bays)
                elif turn_counter == max_number_of_turns:
                    number_of_bay = (number_of_bay - 1) % len(loading_bays)
            
        elif loading_stage == 3:
            # Run the loading pipeline state machine until it either
            # completes a lift or decides to reset the cycle.
            # Reset the state machine for a new loading cycle (ensures clean state for next cycle)
            loading_state.reset_cycle_flags()
            go(0, 0)  # Stop motors
            sleep_ms(300)  # Give sensor time to stabilize after stage 2
            
            result = None
            while True:
                # Check emergency stop before each step
                if emergency_stop:
                    print("Emergency stop detected in loading_stage 3, breaking out")
                    if global_actuator is not None:
                        try:
                            global_actuator.stop()
                        except:
                            pass
                    break
                
                result = pipeline_step(loading_state)
                print("Loading pipeline step result:", result)

                # "waiting_sensor" and "monitoring" mean just keep polling
                if result in ("waiting_sensor", "monitoring", "init_unlocked", "actuator_initialized", "color_sampled"):
                    continue

                # Stop once the lift is done or the state machine resets
                if result in ("lift_triggered", "state_reset"):
                    break

                # Any unexpected result: break to avoid hanging
                print(f"Unexpected result: {result}, breaking out of state machine loop")
                break

            # Get the detected color from state machine (may be None if no color detected)
            colour = loading_state.detected_color
            print("State machine detected color:", colour)

            shift_back_without_correction((16//5.5)*((950//BASE)*40))
            if current_vertex in [V.A_DOWN_BEG, V.B_UP_BEG]:
                spin_right()
                spin_sleep(90)
            elif current_vertex in [V.B_DOWN_BEG, V.A_UP_BEG]:
                spin_left()
                spin_sleep(90)
                turn_counter = max_number_of_turns + 1 - turn_counter
            loading_stage = 4


        elif loading_stage == 4 and (c == 0b1110 or c == 0b1111 or c == 0b0111):
            if turn_counter_on:
                turn_counter += 1
                turn_counter_on = False
            
        elif loading_stage == 4:
            turn_counter_on = True

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
            go(BASE, BASE)
        sleep_ms(DT_MS)
    if current_vertex == V.A_DOWN_BEG:
        current_vertex = V.A_DOWN_END
    elif current_vertex == V.B_UP_BEG:
        current_vertex = V.B_UP_END
    elif loading_stage == 4:
        current_heading = 2
    elif current_vertex == V.B_DOWN_BEG:
        current_vertex = V.B_DOWN_END
    else:
        current_vertex = V.A_UP_END
    if current_vertex != V.A_UP_END:
        shift_with_correction((950//BASE)*40)
    return colour

def complete_route(branch_route):
    global current_heading
    global current_vertex
    global emergency_stop
    branch_index = 0

    # if branch_route[branch_index] == 'R':
    #     spin_right()
    #     spin_sleep(90)
    # elif branch_route[branch_index] == 'L':
    #     spin_left()
    #     spin_sleep(90)
    if branch_route[branch_index] == 'B':
        spin_back()
    if current_vertex == V.A_UP_END:
        shift_back_without_correction((950//BASE)*40)
    branch_index = 1

    turning = None
    fl_cnt = fr_cnt = 0
    stable = 0
    t0 = 0
    while (branch_index < len(branch_route) or not centered(read_code())) and not emergency_stop:
        #print(current_heading)
        c = read_code()
        #print("Code:",bin(c))
        FL = (c>>3)&1;  FR = c&1
        mid = (c>>1)&0b11  # inner pair
                # If all four see white: follow the next route directive
        #print(branch_index)
        
        if c == 0b1111:
            # if we ran out of directives, default to 'F'
            action = branch_route[branch_index] if branch_index < len(branch_route) else 'X'
            print('1')
            if action == 'L':
                print('2')
                turning = 'L'
                t0 = ticks_ms()
                branch_index += arc('L')                 # branch_index += arc() will consume this route entry
                sleep_ms(DT_MS)
                print('3')
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

        # if turning:
        #     # finish when re-centered or inner pair suggests followable track again
        #     if centered(c) or slight_left(c) or slight_right(c):
        #         stable += 1
        #     else:
        #         stable = 0
        #     if stable >= STABLE or ticks_diff(ticks_ms(), t0) > MAX_TURN_MS:
        #         turning = None; stable = 0; go(BASE, BASE); sleep_ms(DT_MS); continue
        #     branch_index += arc(turning); sleep_ms(DT_MS); 
        # # completely useless code, delete after testing



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
    # go(BASE, BASE)
    # sleep_ms(800)
    shift_with_correction((950//BASE)*40)
    current_heading = finish_heading


def go_to(finish_vertex):
    global current_vertex
    
    graph = map.GRAPH
    current_path = map.shortest_path(graph, current_vertex, finish_vertex)
    print('Current heading:', current_heading)
    branch_route = path_to_route(current_path)
    print('Branch route:', branch_route)
    complete_route(branch_route)
    current_vertex = finish_vertex

    print("Path:", current_path)
    print("Route:", branch_route)

if current_vertex in [V.A_DOWN_BEG, V.B_DOWN_BEG, V.A_DOWN_END, V.B_DOWN_END]:
    zone = 'down'
else:
    zone = 'up'
loading_bays = [V.B_DOWN_BEG, V.B_UP_BEG, V.A_UP_BEG, V.B_DOWN_BEG]  # list of loading bay vertices
boxes_delivered = 0
number_of_bay = 0
last_checked_bay = loading_bays[0]

def color_to_vertex(color: str) -> V:
    """
    Map a color string (e.g. 'red') to the corresponding V enum member.

    MicroPython's slim IntEnum fallback doesn't support subscription (V['RED']),
    so we use getattr-style lookup instead.
    """
    key = str(color).upper()
    if hasattr(V, key):
        return getattr(V, key)
    raise ValueError(f"Unknown color: {color!r}")


from linear_actuator import unload_robot


def main():
    global current_vertex
    global last_checked_bay
    global boxes_delivered
    global number_of_bay
    global emergency_stop
    global robot_started
    global init_distance_unlock
    global setup_sensor2
    global loading_state
    global global_actuator

    # This flag controls whether we've already run the actuator init
    # cycle for the current round. It is reset at the end of each loop
    # so that every round starts with a fresh init cycle.
    init_distance_unlock = False

    # Initialize LED state machine but don't start it yet
    sm_yellow = rp2.StateMachine(0, blink_1hz, freq=2000, set_base=Pin(PIN_YELLOW))
    sm_yellow.active(0)  # LED off initially

    # Initialize sensor after a delay to let hardware stabilize
    print("Initializing TMF8701 sensor...")
    sleep_ms(500)  # Give hardware time to stabilize
    setup_sensor2 = setup_sensor_tmf8701()
    loading_state = LoadingPipelineState(tmf8701=setup_sensor2)
    print("Sensor initialization complete.")

    actuator = Actuator(ACTUATOR_DIR_PIN, ACTUATOR_PWM_PIN)
    global_actuator = actuator  # Store global reference for emergency stop
    # loading_pipeline.initialize_actuator_down(actuator)
    actuator_initialized_cycle = True

    # Wait for first button press to start the robot
    print("Waiting for button press to start...")
    while not robot_started:
        sleep_ms(100)  # Wait for button interrupt
    
    # Button pressed: start LED blinking and begin robot operation
    sm_yellow.active(1)
    print("Robot started! LED blinking.")

    while boxes_delivered < 4:

        # If paused by button, stop motors and keep LED off
        if emergency_stop:
            go(0, 0)
            sm_yellow.active(0)  # Stop LED when paused
            # Stop actuator if it exists
            if global_actuator is not None:
                try:
                    global_actuator.stop()
                except:
                    pass
            sleep_ms(100)  # Small delay to avoid busy waiting
            continue
        
        # Robot is running: ensure LED is blinking
        sm_yellow.active(1)

        # Run the actuator initialization cycle once at the start of each round,
        # then proceed to movement on the next iteration.
        if not init_distance_unlock:
            # Check emergency stop before initializing actuator
            if emergency_stop:
                go(0, 0)
                sm_yellow.active(0)
                if global_actuator is not None:
                    try:
                        global_actuator.stop()
                    except:
                        pass
                sleep_ms(100)
                continue
            
            actuator = Actuator(ACTUATOR_DIR_PIN, ACTUATOR_PWM_PIN)
            global_actuator = actuator  # Update global reference
            if number_of_bay in [0, 1]:
                loading_pipeline_state_machine.initialize_actuator_down(actuator)
            else:
                loading_pipeline_state_machine.initialize_actuator_up(actuator)
            init_distance_unlock = True
            emergency_stop = False  # Ensure we continue after initialization
            # Go back to the top of the loop; next iteration will do movement.
            print('0')
            continue

        # Move to the last loading bay spot and check for boxes.
        go_to(last_checked_bay)
        print('1')
        found_color = seek_and_find(loading_bays[number_of_bay])
        # found_color now contains the color detected by state machine in loading_stage == 3
        print("Found color:", found_color)
        if found_color is not None:  # if we found any boxes there
            delivery_area = color_to_vertex(found_color)  # map color to vertex
            print("Delivering to:", delivery_area)
            go_to(delivery_area)  # go to delivery area
            boxes_delivered += 1 # increment boxes delivered
            shift_with_correction((12//5.5)*((950//BASE)*40))
            go(0, 0)
            unload_robot() # unload any boxes we have
            shift_back_without_correction((950//BASE)*40)
            go(0,0)
        else:
            number_of_bay = (number_of_bay + 1) % len(loading_bays) # set target to next bay

        # Prepare for the next round: force the next loop iteration to
        # run the actuator initialization cycle again.
        init_distance_unlock = False


    go_to(V.START)
    shift_to_the_box()
    go(0,0)

def shift_to_the_box():
    pass






# if __name__ == "__main__":
#     unload_robot()


# def main():
#     print("Starting main function")
#     # actuator = Actuator(DIR_PIN, PWM_PIN)
    
#     # # Test retract
#     # actuator.retract(speed=100)
#     # sleep(5)
#     # actuator.stop()
#     # sleep(2)  # Pause so you can measure
    
#     # print("Unloading complete")
    
#     global current_vertex
#     global last_checked_bay
#     global boxes_delivered
#     global number_of_bay

#     while boxes_delivered < 2:

#         go_to(last_checked_bay)
#         print(current_vertex)
#         # we go to last loading bay spot and check if there are any boxes in there. If there are, we pick them up and transport them.
#         if True: #if we found any boxes there
#             go_to(V.GREEN)  # go to delivery area
#             print(1)
#             boxes_delivered += 1 # increment boxes delivered
#             unload_robot() # unload any boxes we have
#             number_of_bay = (number_of_bay + 1) % len(loading_bays) # set target to next bay
#             last_checked_bay = loading_bays[number_of_bay]


#     go_to(V.START)
#     go(0,0)


main()
