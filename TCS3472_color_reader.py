from machine import I2C, Pin
from utime import sleep

I2C_ID = 0
PIN_SDA = 8   # GP8 (SDA)
PIN_SCL = 9   # GP9 (SCL)
POWER_PIN = 0  # GPIO used to gate sensor power (wire through transistor/MOSFET)
POWER_STABILIZE_MS = 50
INTEGRATION_TIME_MS = 760
OFF_TIME_SECONDS = 1.0


def setup_sensor():
    i2c_bus = I2C(id=I2C_ID, sda=Pin(PIN_SDA), scl=Pin(PIN_SCL))

    try:
        from libs.tcs3472 import tcs3472
    except ImportError as e:
        raise RuntimeError("TCS3472 driver not found") from e
    return tcs3472(i2c_bus)


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


def main():
    power_ctrl = Pin(POWER_PIN, Pin.OUT, value=0)

    try:
        while True:
            # Power ON
            power_ctrl.value(1)
            sleep(POWER_STABILIZE_MS / 1000)
            sensor = setup_sensor()
            sleep(INTEGRATION_TIME_MS / 1000)
            light = sensor.light()
            rgb = sensor.rgb()
            color = detect_color(rgb, light)
            print("ON  -> Light:", light, "RGB:", rgb, "Color:", color)

            # Power OFF
            power_ctrl.value(0)
            print("OFF")
            sleep(OFF_TIME_SECONDS)

    except KeyboardInterrupt:
        print("\nStopping TCS3472 reader.")
        power_ctrl.value(0)
    except Exception as e:
        print("Sensor error:", e)
        power_ctrl.value(0)


if __name__ == "__main__":
    main()
