from machine import I2C, Pin
from utime import sleep

# Configuration
I2C_ID = 0
PIN_SCL = 9          # I2C Clock pin (GP9) - adjust if using different pins
PIN_SDA = 8          # I2C Data pin (GP8) - adjust if using different pins
SAMPLE_INTERVAL_SECONDS = 0.5


def setup_sensor():
    i2c_bus = I2C(id=I2C_ID, sda=Pin(PIN_SDA), scl=Pin(PIN_SCL))

    try:
        from libs.VL53L0X.VL53L0X import VL53L0X
    except ImportError as e:
        raise RuntimeError("VL53L0X driver not found") from e

    sensor = VL53L0X(i2c_bus)

    sensor.set_Vcsel_pulse_period(sensor.vcsel_period_type[0], 18)
    sensor.set_Vcsel_pulse_period(sensor.vcsel_period_type[1], 14)

    return sensor


def read_distance_mm(sensor) -> int:
    return sensor.read()


def main():
    sensor = setup_sensor()
    
    while True:
        try:
            # Start device for measurement
            sensor.start()
            
            # Read distance
            dist_mm = read_distance_mm(sensor)
            
            if dist_mm is None:
                print("Distance: N/A")
            else:
                # Convert to cm for display
                dist_cm = dist_mm / 10.0
                print(f"Distance: {dist_mm}mm ({dist_cm:.1f}cm)")
            
            # Stop device (optional, can keep running)
            sensor.stop()
            
            sleep(SAMPLE_INTERVAL_SECONDS)
            
        except KeyboardInterrupt:
            print("\nStopping VL53L0X reader.")
            sensor.stop()  # Ensure sensor is stopped
            break
        except Exception as e:
            print(f"Sensor error: {e}")
            sensor.stop()  # Ensure sensor is stopped on error
            sleep(1.0)


if __name__ == "__main__":
    main()

