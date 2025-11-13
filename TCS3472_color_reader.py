from machine import I2C, Pin
from utime import sleep

try:
    from libs.tcs3472 import tcs3472
except ImportError:
    from TCS3472 import tcs3472


I2C_ID = 0
PIN_SDA = 4
PIN_SCL = 5
SAMPLE_INTERVAL_SECONDS = 1.0


def setup_sensor():
    i2c_bus = I2C(id=I2C_ID, sda=Pin(PIN_SDA), scl=Pin(PIN_SCL))
    return tcs3472(i2c_bus)


def main():
    sensor = setup_sensor()
    print("TCS3472 color sensor reader (every 1s). Ctrl+C to stop.")

    while True:
        try:           
            light = sensor.light()
            rgb = sensor.rgb()
            print("Light:", light)
            print("RGB:", rgb)
            sleep(SAMPLE_INTERVAL_SECONDS)
        except KeyboardInterrupt:
            print("\nStopping TCS3472 reader.")
            break
        except Exception as exc:
            print("Sensor error:", exc)
            sleep(1.0)


if __name__ == "__main__":
    main()

