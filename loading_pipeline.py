from machine import I2C, Pin
from utime import sleep

from libs.tcs3472 import tcs3472
from linear_actuator import Actuator
from libs.tmf8701 import DFRobot_TMF8701
from libs.VL53L0X import VL53L0X

# Zone configuration: "zone_down" or "zone_up"
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
INIT_RETRACT_TIME = 3.0  # Retract to bottommost position
ZONE_DOWN_EXTEND_TIME = 8.0  # Extend to default position for zone_down
ZONE_UP_EXTEND_TIME = 0  # Extend to default position for zone_up
LIFT_TIME = 3.0  # Base lift time when starting loading
ACTUATOR_SPEED = 50

MIN_INIT_DISTANCE_CM = 5
ZONE_PRESET_DISTANCE_CM = 10.0 # Trigger zone preset extend
COLOR_REFERENCE_DISTANCE_CM = 3.0  # Trigger color sampling
LIFT_REFERENCE_DISTANCE_CM = 2.0  # Trigger lift phase
LOOP_DELAY = 0.2

def get_zone_extend_time():
    if LOADING_ZONE == 1:
        return ZONE_DOWN_EXTEND_TIME
    if LOADING_ZONE == 2:
        return ZONE_UP_EXTEND_TIME
    return 0.0
    
def initialize_actuator(actuator):
    # actuator.retract(speed=100)  # Maximum speed
    # sleep(INIT_RETRACT_TIME)
    # actuator.stop()
    # sleep(0.1)
    # print("Reached bottommost position")
    # # Set default position based on zone
    # zone_extend_time = get_zone_extend_time()
    
    print("Setting default position for zone {} (extending for {} seconds)...".format(LOADING_ZONE, 6))
    actuator.retract(speed=ACTUATOR_SPEED)
    sleep(6)
    actuator.stop()
    sleep(0.1)
    global actuator_initialized_cycle
    actuator_initialized_cycle = True
    print("Actuator initialization complete. Ready for loading.\n")



def perform_lift(actuator):
    print("Lift phase: extending actuator for {}s at speed {}.".format(LIFT_TIME, ACTUATOR_SPEED))
    actuator.extend(speed=ACTUATOR_SPEED)
    sleep(LIFT_TIME)
    actuator.stop()
    sleep(0.5)
    print("Lift phase complete.")

def setup_sensor_tmf8701():
    i2c = I2C(I2C_ID, sda=Pin(PIN_SDA), scl=Pin(PIN_SCL))
    sensor = DFRobot_TMF8701(i2c)
    if sensor.begin() != 0:
        raise RuntimeError("TMF8701 initialization failed")
    if not sensor.start_measurement(sensor.eMODE_NO_CALIB, sensor.eCOMBINE):
        raise RuntimeError("TMF8701 failed to start measurement")
    print("TMF8701 distance reader ready.")
    return sensor

def setup_sensor_tcs3472():
    i2c_bus_tcs3472 = I2C(I2C_ID_TCS3472, sda=Pin(PIN_SDA_TCS3472), scl=Pin(PIN_SCL_TCS3472))
    sensor = tcs3472(i2c_bus_tcs3472)
    return sensor

def detect_color(rgb, light):
    r, g, b = rgb
    if light < 50 or (r == 0 and g == 0 and b == 0):
        return "DARK"
    total = r + g + b
    r_ratio = r / total
    g_ratio = g / total
    b_ratio = b / total

    if r_ratio > 0.33 and g_ratio > 0.33:
        return "YELLOW"
    if r_ratio > 0.33 and r > 100:
        return "RED"
    if g_ratio > 0.33 and g > 80:
        return "GREEN"
    if b_ratio > 0.33 and b > 100:
        return "BLUE"

    return "UNKNOWN"
    
def sample_color(power_ctrl):
    power_ctrl.value(1)
    sleep(COLOR_POWER_STABILIZE_MS / 1000)

    sensor = setup_sensor_tcs3472()
    sleep(COLOR_INTEGRATION_MS/1000)
    light = sensor.light()
    rgb = sensor.rgb()
    power_ctrl.value(0)
    sleep(LOOP_DELAY)
    return light, rgb

def read_distance_cm(sensor):
    if sensor.is_data_ready():
        dist_mm = sensor.get_distance_mm()
        return dist_mm / 10.0
    return None


def loading_pipeline_main():
    # Power control
    color_power = Pin(COLOR_POWER_PIN, Pin.OUT, value=0)
    
    # Sensors
    tmf8701 = setup_sensor_tmf8701()

    # Actuator
    actuator = Actuator(ACTUATOR_DIR_PIN, ACTUATOR_PWM_PIN)

    color_sampled = False
    lift_triggered = False
    detected_color = None
    dist_cm = None

    while True:
        dist_cm = read_distance_cm(tmf8701)

        if LOADING_ZONE not in (1, 2):
            print("Not in loading zone 1 or 2, stopping loading pipeline.")
            break

        if dist_cm is None:
            print("Distance: -- waiting for sensor data")
            sleep(LOOP_DELAY)
            continue

        if (
            dist_cm <= COLOR_REFERENCE_DISTANCE_CM
            and actuator_initialized_cycle
            and not color_sampled
        ):
            light, rgb = sample_color(color_power)
            color = detect_color(rgb, light)
            print("Light:", light, "RGB:", rgb, "Detected color:", color)
            color_sampled = True
            detected_color = color
            sleep(LOOP_DELAY)
            continue

        if (
            dist_cm <= LIFT_REFERENCE_DISTANCE_CM
            and actuator_initialized_cycle
            and color_sampled
            and not lift_triggered
        ):
            print(
                "Starting lift: last detected color {}."
                .format(detected_color or "UNKNOWN")
            )
            perform_lift(actuator)
            lift_triggered = True
            sleep(LOOP_DELAY)
            print("Loading cycle finished. Exiting loading loop.")
            break

        if dist_cm > ZONE_PRESET_DISTANCE_CM and (actuator_initialized_cycle or color_sampled or lift_triggered):
            print("Object moved out of range. Resetting cycle state.")
            actuator_initialized_cycle = False
            color_sampled = False
            lift_triggered = False
            detected_color = None
            init_distance_unlock = False

        sleep(LOOP_DELAY)
