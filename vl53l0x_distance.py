from machine import I2C, Pin
from utime import sleep

I2C_ID = 0
PIN_SCL = 9          
PIN_SDA = 8          
SAMPLE_INTERVAL_SECONDS = 0.5


def setup_sensor():
    i2c_bus = I2C(id=I2C_ID, sda=Pin(PIN_SDA), scl=Pin(PIN_SCL))

    try:
        from libs.VL53L0X.VL53L0X import VL53L0X
    except ImportError as e:
        raise RuntimeError("VL53L0X driver not found") from e
    sensor = VL53L0X(i2c_bus)
    #Higher numbers = longer pulse → more light → potentially more range, 
    #but slower measurement and higher power.
    #Pre-range 2,14,16,18; Final range 1, 12, 14, 16
    sensor.set_Vcsel_pulse_period(sensor.vcsel_period_type[0], 18) 
    sensor.set_Vcsel_pulse_period(sensor.vcsel_period_type[1], 14)

    return sensor


def vl5310x_read_distance(sensor) -> int:
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

