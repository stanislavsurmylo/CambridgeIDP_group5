from machine import I2C, Pin
from utime import sleep_ms
from libs.vl53l0x import VL53L0X, TimeoutError

I2C_ID = 0
PIN_SCL = 9          
PIN_SDA = 8          

def setup_sensor_vl53l0x():
    """Initialize VL53L0X distance sensor (single attempt)."""
    try:
        sleep_ms(200)
        i2c_bus = I2C(id=I2C_ID, sda=Pin(PIN_SDA), scl=Pin(PIN_SCL), freq=100000)
        sleep_ms(50)
        
        devices = i2c_bus.scan()
        if 0x29 not in devices:
            print("ERROR: Sensor not found at 0x29")
            return None
        
        sleep_ms(100)
        sensor = VL53L0X(i2c_bus)
        sensor.set_Vcsel_pulse_period(sensor.vcsel_period_type[0], 18)
        sensor.set_Vcsel_pulse_period(sensor.vcsel_period_type[1], 14)
        sensor.start()
        return sensor
    except Exception as e:
        print(f"Setup error: {e}")
        return None


def reinit_sensor_vl53l0x(sensor=None):
    """Re-initialize VL53L0X sensor. Stops existing sensor if provided, then creates a new one."""
    try:
        # Stop existing sensor if provided
        if sensor is not None:
            try:
                sensor.stop()
                print("Stopped existing sensor")
            except:
                pass
        
        # Wait a bit before reinitializing
        sleep_ms(200)
        
        # Create new I2C bus
        i2c_bus = I2C(id=I2C_ID, sda=Pin(PIN_SDA), scl=Pin(PIN_SCL), freq=100000)
        sleep_ms(50)
        
        # Check if sensor is present
        devices = i2c_bus.scan()
        print(f"Found I2C devices: {[hex(d) for d in devices]}")
        if 0x29 not in devices:
            print("ERROR: Sensor not found at 0x29")
            return None
        
        sleep_ms(100)
        print("Re-initializing sensor...")
        sensor = VL53L0X(i2c_bus)
        sensor.set_Vcsel_pulse_period(sensor.vcsel_period_type[0], 18)
        sensor.set_Vcsel_pulse_period(sensor.vcsel_period_type[1], 14)
        sensor.start()  # Start the sensor immediately
        print("Sensor re-initialized and started")
        return sensor
    except Exception as e:
        print(f"Re-initialization error: {e}")
        return None


def vl53l0x_read_distance(sensor):
    """Read distance from VL53L0X sensor. Returns distance in mm, or None if error."""
    try:
        dist_mm = sensor.read()
        # Allow readings from 0 to 2000mm (remove lower limit check)
        if dist_mm >= 8190 or dist_mm > 2000:
            return None
        return dist_mm
    except (TimeoutError, Exception) as e:
        return None


def main():
    sensor = setup_sensor_vl53l0x()
    if sensor is None:
        return
    
    try:
        while True:
            dist_mm = vl53l0x_read_distance(sensor)
            if dist_mm is not None:
                print(dist_mm)
            sleep_ms(100)
    except KeyboardInterrupt:
        pass
    finally:
        sensor.stop()


if __name__ == "__main__":
    main()
