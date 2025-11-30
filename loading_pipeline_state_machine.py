from machine import I2C, Pin
from utime import sleep

from libs.tcs3472 import tcs3472
from linear_actuator import Actuator
from libs.tmf8701 import DFRobot_TMF8701

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
COLOR_POWER_STABILIZE_MS = 2000
COLOR_INTEGRATION_MS =2000

# Actuator pins
ACTUATOR_DIR_PIN = 3
ACTUATOR_PWM_PIN = 2

# Timing constants
INIT_RETRACT_TIME = 5.0  # Retract to bottommost position
ZONE_DOWN_EXTEND_TIME = 6.3 # Extend to default position for zone_down
ZONE_UP_EXTEND_TIME = 6.3  # Extend to default position for zone_up
LIFT_TIME = 3.0  # Base lift time when starting loading
ACTUATOR_SPEED = 50

MIN_INIT_DISTANCE_CM = 5
ZONE_PRESET_DISTANCE_CM = 10.0 # Trigger zone preset extend
COLOR_REFERENCE_DISTANCE_CM = 5  # Trigger color sampling
LIFT_REFERENCE_DISTANCE_CM = 3 # Trigger lift phase
LOOP_DELAY = 0.2

def get_zone_extend_time(zone=LOADING_ZONE):
    if zone == 1:
        return ZONE_DOWN_EXTEND_TIME
    if zone == 2:
        return ZONE_UP_EXTEND_TIME
    return 0.0
    
def initialize_actuator_down(actuator, zone=LOADING_ZONE):
    actuator.retract(speed=100)  # Maximum speed
    sleep(INIT_RETRACT_TIME)
    actuator.stop()
    sleep(0.1)
    print("Reached bottommost position")
    # Set default position based on zone
    zone_extend_time = get_zone_extend_time(zone)
    print("Setting default position for zone {} (extending for {} seconds)...".format(zone, zone_extend_time))
    actuator.extend(speed=ACTUATOR_SPEED)
    sleep(zone_extend_time)
    actuator.stop()
    sleep(0.1)
    print("Actuator initialization complete. Ready for loading.\n")

def initialize_actuator_up(actuator, zone=LOADING_ZONE):
    actuator.retract(speed=100)  # Maximum speed
    sleep(INIT_RETRACT_TIME)
    actuator.stop()
    sleep(0.1)
    print("Reached bottommost position")
    # Set default position based on zone
    zone_extend_time = get_zone_extend_time(zone)
    print("Setting default position for zone {} (extending for {} seconds)...".format(zone, zone_extend_time - 5.5))
    actuator.extend(speed=ACTUATOR_SPEED)
    sleep(zone_extend_time - 5.5)
    actuator.stop()
    sleep(0.1)
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
    
def sample_color(power_ctrl, tcs3472_sensor=None):
    try:
        power_ctrl.value(1)
        sleep(COLOR_POWER_STABILIZE_MS / 1000)

        # Use provided sensor or create new one
        if tcs3472_sensor is None:
            sensor = setup_sensor_tcs3472()
        else:
            sensor = tcs3472_sensor
        sleep(COLOR_INTEGRATION_MS/1000)
        light = sensor.light()
        rgb = sensor.rgb()
        power_ctrl.value(0)
        sleep(LOOP_DELAY)
        return light, rgb
    except Exception as e:
        # Handle I2C timeout or other communication errors
        print(f"I2C error in sample_color: {e}")
        power_ctrl.value(0)  # Make sure to turn off power on error
        return (0, (0, 0, 0))  # Return default values

def read_distance_cm(sensor):
    try:
        if sensor.is_data_ready():
            dist_mm = sensor.get_distance_mm()
            return dist_mm / 10.0
    except Exception as e:
        # Handle I2C timeout or other communication errors
        print(f"I2C error in read_distance_cm: {e}")
        return None
    return None


class LoadingPipelineState:
    """Holds hardware references and per-cycle flags for the loading pipeline."""

    def __init__(self, loading_zone=LOADING_ZONE, loop_delay=LOOP_DELAY, tmf8701=None, actuator=None, tcs3472=None):
        self.loading_zone = loading_zone
        self.loop_delay = loop_delay
        self.color_power = Pin(COLOR_POWER_PIN, Pin.OUT, value=0)
        # Use provided sensor/actuator or create new ones
        self.tmf8701 = tmf8701 if tmf8701 is not None else setup_sensor_tmf8701()
        self.actuator = actuator if actuator is not None else Actuator(ACTUATOR_DIR_PIN, ACTUATOR_PWM_PIN)
        # TCS3472 color sensor (can be None, will be created on demand if needed)
        self.tcs3472 = tcs3472
        self.reset_cycle_flags()

    def reset_cycle_flags(self):
        # In this version we assume the actuator has already been
        # mechanically initialised by higher‑level code (e.g. main.py),
        # so we start each cycle "unlocked" and ready to run the colour
        # + lift phases as soon as distance thresholds are met.
        self.actuator_initialized_cycle = True
        self.color_sampled = False
        self.lift_triggered = False
        self.detected_color = None
        self.init_distance_unlock = True


def _maybe_delay(state):
    if getattr(state, "loop_delay", 0):
        sleep(state.loop_delay)


def pipeline_step(state):
    """
    Execute a single iteration of the loading pipeline state machine.

    Returns a string describing what happened (useful for higher-level schedulers).
    """
    try:
        dist_cm = read_distance_cm(state.tmf8701)

        # Only treat 'None' as no data; accept 0 or any numeric value so
        # the state machine can still progress with your real sensor output.
        if dist_cm is None:
            print("Distance: -- waiting for sensor data")
            _maybe_delay(state)
            return "waiting_sensor"

        # Debug: show the raw distance the state machine is seeing
        print("State machine distance (cm):", dist_cm)

        if (
            dist_cm <= COLOR_REFERENCE_DISTANCE_CM
            and state.init_distance_unlock
            and not state.color_sampled
        ):
            try:
                light, rgb = sample_color(state.color_power, state.tcs3472)
                color = detect_color(rgb, light)
                print("Light:", light, "RGB:", rgb, "Detected color:", color)
                state.color_sampled = True
                state.detected_color = color
                _maybe_delay(state)
                return "color_sampled"
            except Exception as e:
                print(f"Error in pipeline_step (color sampling): {e}")
                _maybe_delay(state)
                return "monitoring"  # Continue monitoring even if color sampling fails

        if (
            dist_cm <= LIFT_REFERENCE_DISTANCE_CM
            and state.init_distance_unlock
            and state.color_sampled
            and not state.lift_triggered
        ):
            try:
                print(
                    "Starting lift: last detected color {}."
                    .format(state.detected_color or "UNKNOWN")
                )
                perform_lift(state.actuator)
                state.lift_triggered = True
                _maybe_delay(state)
                return "lift_triggered"
            except Exception as e:
                print(f"Error in pipeline_step (performing lift): {e}")
                _maybe_delay(state)
                return "monitoring"  # Continue monitoring even if lift fails

        if dist_cm > ZONE_PRESET_DISTANCE_CM and (
            state.init_distance_unlock or state.color_sampled or state.lift_triggered
        ):
            print("Object moved out of range. Resetting cycle state.")
            state.reset_cycle_flags()
            _maybe_delay(state)
            return "state_reset"

        _maybe_delay(state)
        return "monitoring"
    except Exception as e:
        print(f"Error in pipeline_step (general): {e}")
        _maybe_delay(state)
        return "waiting_sensor"


def main():
    """
    Standalone runner for the loading pipeline state machine.
    Runs until a single loading cycle has either completed a lift
    or been reset, then returns to the caller.
    """
    state = LoadingPipelineState()
    while True:
        result = pipeline_step(state)
        if result in ("lift_triggered", "state_reset"):
            # One complete cycle done; exit so caller (e.g. main.py)
            # can continue with the rest of the robot logic.
            break

if __name__ == "__main__":
    main()