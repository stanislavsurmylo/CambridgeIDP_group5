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

def unload_robot():

    # Unloading robot to reduce weight on actuator
    actuator = Actuator(DIR_PIN, PWM_PIN)
    
    # Test retract
    actuator.retract(speed=100)
    sleep(5)
    actuator.stop()
    sleep(2)  # Pause so you can measure
    
    print("Unloading complete")


if __name__ == "__main__":
    test_actuator()