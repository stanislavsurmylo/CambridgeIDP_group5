from machine import Pin, PWM
from utime import sleep

class Actuator:
    def __init__(self, dirPin, PWMPin):
        self.mDir = Pin(dirPin, Pin.OUT)  # set motor direction pin
        self.pwm = PWM(Pin(PWMPin))  # set motor pwm pin
        self.pwm.freq(1000)  # set PWM frequency
        self.pwm.duty_u16(0)  # set duty cycle - 0=off
           
    def set(self, dir, speed):
        self.mDir.value(dir)                     # forward = 0 reverse = 1 motor
        self.pwm.duty_u16(int(65535 * speed / 100))  # speed range 0-100 motor
    
    def stop(self):
        self.pwm.duty_u16(0)  # Stop the actuator
    
    def extend(self, speed=50):
        """Extend the actuator at specified speed (0-100)"""
        self.set(dir=0, speed=speed)
    
    def retract(self, speed=50):
        """Retract the actuator at specified speed (0-100)"""
        self.set(dir=1, speed=speed)

DIR_PIN = 3
PWM_PIN = 2
TEST_DURATION = 1.0  # 1 second for testing

def test_actuator():
    """Test function: extend for 1 second, then retract for 1 second"""
    actuator = Actuator(DIR_PIN, PWM_PIN)
    
    print("Starting actuator test...")
    print("=" * 40)
    
    # Test extend
    print("Extending for {} seconds...".format(TEST_DURATION))
    actuator.extend(speed=50)
    sleep(TEST_DURATION)
    actuator.stop()
    print("Extended - measure distance now")
    sleep(2)  # Pause so you can measure
    
    # Test retract
    print("\nRetracting for {} seconds...".format(TEST_DURATION))
    actuator.retract(speed=50)
    sleep(TEST_DURATION)
    actuator.stop()
    print("Retracted - measure distance now")
    sleep(2)  # Pause so you can measure
    
    print("\nTest complete!")


if __name__ == "__main__":
    test_actuator()