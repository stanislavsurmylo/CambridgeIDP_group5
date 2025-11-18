from machine import I2C, Pin, UART
from utime import sleep


I2C_ID = 0
PIN_SDA = 8   # GP8 (SDA)
PIN_SCL = 9   # GP9 (SCL)
UART_ID = 0   # UART0 for start/stop commands
UART_BAUD = 115200
SAMPLE_INTERVAL_SECONDS = 0.5


def setup_sensor():
    i2c_bus = I2C(id=I2C_ID, sda=Pin(PIN_SDA), scl=Pin(PIN_SCL))

    try:
        from libs.tcs3472 import tcs3472
    except ImportError as e:
        raise RuntimeError ("TCS3472 driver not found") from e
    return tcs3472(i2c_bus)

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

def main():
    sensor = setup_sensor()
    uart = UART(UART_ID, baudrate=UART_BAUD)
    sensor_enabled = False

    print("TCS3472 color reader ready.")
    print("Send '1' over UART0 to START, '0' to STOP.")

    while True:
        try:
            if uart.any():
                cmd = uart.read(1)
                if cmd == b'1':
                    sensor_enabled = True
                    print("Color sensor ENABLED")
                elif cmd == b'0':
                    sensor_enabled = False
                    print("Color sensor DISABLED")
            if sensor_enabled:
                light = sensor.light()
                rgb = sensor.rgb()
                color = detect_color(rgb, light)
                print("Light:", light)
                print("RGB:", rgb)
                print("Color:", color)
            else:
                print("Sensor OFF (waiting for UART command '1')")
            sleep(SAMPLE_INTERVAL_SECONDS)
        except KeyboardInterrupt:
            print("\nStopping TCS3472 reader.")
            break
        except Exception as e:
            print("Sensor error:", e)
            sleep(1.0)

if __name__ == "__main__":
    main()

