from machine import I2C, Pin
from utime import sleep


I2C_ID = 0
PIN_SDA = 8   
PIN_SCL = 9   
BUTTON_PIN = 16  
SAMPLE_INTERVAL_SECONDS = 0.5


def setup_sensor():
    i2c_bus = I2C(id=I2C_ID, sda=Pin(PIN_SDA), scl=Pin(PIN_SCL))

    try:
        from libs.tcs3472 import tcs3472
    except ImportError as e:
        raise RuntimeError ("TCS3472 driver not found") from e
    return tcs3472(i2c_bus)

def detect_color(rgb, light):
    """Return a basic color name based on RGB proportions."""
    r, g, b = rgb

    if light < 50 or (r == 0 and g == 0 and b == 0):
        return "DARK"
    total = r + g + b
    r_ratio = r / total
    g_ratio = g / total
    b_ratio = b / total

    if r_ratio > 0.33 and r > 100:
        if g_ratio > 0.3:
            return "YELLOW"  
        return "RED"
    if g_ratio > 0.33 and g > 80:
        return "GREEN"
    if b_ratio > 0.45 and b > 100:
        return "BLUE"
    if r_ratio > 0.33 and g_ratio > 0.33:
        return "YELLOW"
    return "UNKNOWN"

def main():
    sensor = setup_sensor()
    button = Pin(BUTTON_PIN, Pin.IN, Pin.PULL_UP)

    while True:
        try:
            pressed = (button.value() == 0)
            if pressed:
                light = sensor.light()
                rgb = sensor.rgb()
                color = detect_color(rgb, light)
                print("Light:", light)
                print("RGB:", rgb)
                print("Color:", color)
            else:
                print("Button OFF")
            sleep(SAMPLE_INTERVAL_SECONDS)
        except KeyboardInterrupt:
            print("\nStopping TCS3472 reader.")
            break
        except Exception as e:
            print("Sensor error:", e)
            sleep(1.0)

if __name__ == "__main__":
    main()

