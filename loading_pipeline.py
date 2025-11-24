from machine import I2C, Pin
from utime import sleep

from libs.tcs3472 import tcs3472
from linear_actuator import Actuator

# I2C pins
I2C_ID = 0
PIN_SDA = 8
PIN_SCL = 9

# TCS3472 power control pin
COLOR_POWER_PIN = 0

# Actuator pins
ACTUATOR_DIR_PIN = 3
ACTUATOR_PWM_PIN = 2

# Zone configuration: "zone_down" or "zone_up"
LOADING_ZONE = 1  # zone down = 1, zone up = 2

# Timing constants
INIT_RETRACT_TIME = 5.0  # Retract to bottommost position
ZONE_DOWN_EXTEND_TIME = 8.0  # Extend to default position for zone_down
LIFT_TIME = 3.0  # Lift time when starting loading
ACTUATOR_SPEED=50

REFERENCE_DISTANCE_CM = 3.0
LOOP_DELAY = 0.2

def setup_sensor():
    i2c_bus = I2C(id=I2C_ID, sda=Pin(PIN_SDA), scl=Pin(PIN_SCL))

    try:
        from libs.vl53l0x import VL53L0X
    except ImportError as e:
        raise RuntimeError("VL53L0X driver not found") from e
    sensor = VL53L0X(i2c_bus)
    #Higher numbers = longer pulse → more light → potentially more range, 
    #but slower measurement and higher power.
    #Pre-range 2,14,16,18; Final range 1, 12, 14, 16
    sensor.set_Vcsel_pulse_period(sensor.vcsel_period_type[0], 18) 
    sensor.set_Vcsel_pulse_period(sensor.vcsel_period_type[1], 14)

    return sensor

def detect_color(rgb, light):
    r, g, b = rgb
    if light < 50 or (r == 0 and g == 0 and b == 0):
        return "DARK"
    total = r + g + b
    r_ratio = r / total
    g_ratio = g / total
    b_ratio = b / total

    if r_ratio > 0.33 and r > 100:
        return "RED"
    if g_ratio > 0.33 and g > 80:
        return "GREEN"
    if b_ratio > 0.33 and b > 100:
        return "BLUE"
    if r_ratio > 0.33 and g_ratio > 0.33:
        return "YELLOW"
    return "UNKNOWN"

def read_distance_cm(sensor):
    sensor.start()
    try:
        dist_mm = sensor.read()
    finally:
        sensor.stop()
    if dist_mm is None:
        return None
    return dist_mm / 10.0

def initialize_actuator(actuator):
    actuator.retract(speed=100)  # Maximum speed
    sleep(INIT_RETRACT_TIME)
    actuator.stop()
    print("Reached bottommost position")
    
    # Set default position based on zone
    if LOADING_ZONE == 1:
        print("Setting default position for zone_down (extending for {} seconds)...".format(ZONE_DOWN_EXTEND_TIME))
        actuator.extend(speed=ACTUATOR_SPEED)
        sleep(ZONE_DOWN_EXTEND_TIME)
        actuator.stop()
        print("Default position set for zone_down")
    elif LOADING_ZONE == 2:
        print("Zone up: already at default position, no adjustment needed")
    else:
        print("Warning: Unknown zone '{}', skipping default position setup".format(LOADING_ZONE))
    
    print("Actuator initialization complete. Ready for loading.\n")

def main():
    # Shared I2C bus
    i2c_bus = I2C(I2C_ID, sda=Pin(PIN_SDA), scl=Pin(PIN_SCL))
    color_power = Pin(COLOR_POWER_PIN, Pin.OUT, value=0)
    
    # Sensors
    vl53 = setup_sensor()

    # Actuator
    actuator = Actuator(ACTUATOR_DIR_PIN, ACTUATOR_PWM_PIN)
    
    # Initialize actuator to default position
    initialize_actuator(actuator)

    triggered = False
    print("Loading pipeline started. Zone: {}. Waiting for object within {} cm...".format(LOADING_ZONE, REFERENCE_DISTANCE_CM))

    while True:
        dist_cm = read_distance_cm(vl53)
        if dist_cm is not None:
            print("Distance: {:.2f} cm".format(dist_cm))

            if dist_cm <= REFERENCE_DISTANCE_CM and not triggered:
                # Power up TCS3472 only when needed
                color_power.value(1)
                sleep(1)
                color_sensor = tcs3472(i2c_bus)
                light = color_sensor.light()
                rgb = color_sensor.rgb()
                color = detect_color(rgb, light)
                color_power.value(0)
                print("Light:", light, "RGB:", rgb, "Detected color:", color)

                # Start loading: lift at 50 speed for 3 seconds
                print("Starting loading: lifting actuator at 50 speed for {} seconds".format(LIFT_TIME))
                actuator.extend(speed=ACTUATOR_SPEED)
                sleep(LIFT_TIME)
                actuator.stop()
                print("Loading lift complete")
                triggered = True

            elif dist_cm > REFERENCE_DISTANCE_CM + 1.0:
                triggered = False

        sleep(LOOP_DELAY)

if __name__ == "__main__":
    main()

