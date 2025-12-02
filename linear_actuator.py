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
TEST_DURATION = 7.3  # 1 second for testing

def unload_robot():

    DIR_PIN = 3
    PWM_PIN = 2
    TEST_DURATION = 12  # 1 second for testing
    

    # Unloading robot to reduce weight on actuator
    actuator = Actuator(DIR_PIN, PWM_PIN)
    
    # Test retract
    print("Retracting for {} seconds...".format(TEST_DURATION))
    actuator.retract(speed=70)
    sleep(TEST_DURATION)
    actuator.stop()
    print("Unloading complete")
    sleep(0.2)  # Pause so you can measure


if __name__ == "__main__":
    unload_robot()