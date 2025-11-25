from machine import I2C, Pin
from utime import sleep
from libs.tmf8701 import DFRobot_TMF8701

I2C_ID = 0
PIN_SDA = 8     # GP8
PIN_SCL = 9     # GP9
SAMPLE_INTERVAL_SECONDS = 0.5

def tmf8701_read_distance(sensor):
    pass

def setup_sensor():
    pass

def main():
    i2c = I2C(I2C_ID, sda=Pin(PIN_SDA), scl=Pin(PIN_SCL))
    sensor = DFRobot_TMF8701(i2c)

    if sensor.begin() != 0:
        raise RuntimeError("TMF8701 initialization failed")

    if not sensor.start_measurement(sensor.eMODE_CALIB, sensor.eDISTANCE):
        raise RuntimeError("TMF8701 failed to start measurement")

    print("TMF8701 distance reader ready.")

    try:
        while True:
            if sensor.is_data_ready():
                dist_mm = sensor.get_distance_mm()
                print(f"Distance: {dist_mm} mm ({dist_mm / 10.0:.1f} cm)")
            else:
                print("Distance: waiting...")
            sleep(SAMPLE_INTERVAL_SECONDS)
    except KeyboardInterrupt:
        print("\nStopping TMF8701 reader.")
        sensor.stop_measurement()
    except Exception as e:
        print("Sensor error:", e)
        sensor.stop_measurement()
        raise


if __name__ == "__main__":
    main()

