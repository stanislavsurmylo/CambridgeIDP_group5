from machine import I2C, Pin
from utime import sleep

I2C_ID = 0
PIN_SCL = 9          
PIN_SDA = 8          
SAMPLE_INTERVAL_SECONDS = 0.5

try:
    import VL53L0X  # this imports the file VL53L0X.py as a module
except ImportError as e:
    raise RuntimeError(
        "VL53L0X driver not found. Make sure 'VL53L0X.py' is on the Pico."
    ) from e
# --- import the driver once, and show a clear error if something is wrong ---
# try:
#     import VL53L0X as vl
# except ImportError as e:
#     # This means the file VL53L0X.py is not found on the Pico or cannot be imported
#     raise RuntimeError(
#         "Cannot import VL53L0X module. "
#         "Check that 'VL53L0X.py' is copied to the Pico (/, or /lib) "
#         "and the name/case matches exactly."
#     ) from e

# # figure out how the class is named inside the driver
# VLClass = getattr(vl, "VL53L0Xclass", None) or getattr(vl, "VL53L0X", None)
# if VLClass is None:
#     # Module is there, but the expected class is missing
#     raise RuntimeError(
#         "VL53L0X driver mod   ule imported, but neither 'VL53L0Xclass' nor "
#         "'VL53L0X' is defined inside it."
#     )

def setup_sensor():
    i2c_bus = I2C(id=I2C_ID, sda=Pin(PIN_SDA), scl=Pin(PIN_SCL))

    # class lives inside the module as VL53L0X.VL53L0Xclass
    sensor = VL53L0X.VL53L0Xclass(i2c_bus)

    # Higher numbers = longer pulse → more light → potentially more range
    sensor.set_Vcsel_pulse_period(sensor.vcsel_period_type[0], 18)
    sensor.set_Vcsel_pulse_period(sensor.vcsel_period_type[1], 14)

    return sensor


def vl53l0x_read_distance(sensor) -> int:
    return sensor.read()

def main():
    sensor = setup_sensor()
    
    while True:
        try:
            sensor.start()
            
            dist_mm = vl5310x_read_distance(sensor)
            
            if dist_mm is None:
                print("Distance: N/A")
            else:
                dist_cm = dist_mm / 10.0
                print(f"Distance: {dist_mm}mm ({dist_cm:.1f}cm)")
            
            sensor.stop()
            sleep(SAMPLE_INTERVAL_SECONDS)
            
        except KeyboardInterrupt:
            print("\nStopping VL53L0X reader.")
            sensor.stop()  
            break
        except Exception as e:
            print(f"Sensor error: {e}")
            sensor.stop()  
            sleep(1.0)


if __name__ == "__main__":
    main()

