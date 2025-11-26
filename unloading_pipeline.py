from utime import sleep
from linear_actuator import Actuator
from machine import Pin

# Reuse the same actuator pins as loading_pipeline.py
ACTUATOR_DIR_PIN = 3
ACTUATOR_PWM_PIN = 2

UNLOAD_RETRACT_TIME = 5.0   # seconds
UNLOAD_SPEED = 100          # percent (0–100)


def unload_once():
    """
    Basic unloading pipeline:
    - Create an Actuator on the standard pins
    - Retract at full speed for UNLOAD_RETRACT_TIME seconds
    - Then stop
    """
    print("Unloading: retracting actuator for {}s at speed {}."
          .format(UNLOAD_RETRACT_TIME, UNLOAD_SPEED))

    actuator = Actuator(ACTUATOR_DIR_PIN, ACTUATOR_PWM_PIN)
    actuator.retract(speed=UNLOAD_SPEED)
    sleep(UNLOAD_RETRACT_TIME)
    actuator.stop()
    print("Unloading complete; actuator stopped.")


def main():
    unload_once()


if __name__ == "__main__":
    main()


