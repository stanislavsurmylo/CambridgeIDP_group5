from machine import Pin, PWM
from time import sleep

SENSOR_PIN1 = 26           # GP26 (pin 31)
SENSOR_PIN2 = 27           # GP27 (pin 32)
SENSOR_PIN3 = 28           # GP28 (pin 34)
led = Pin("LED", Pin.OUT) # on-board LED
sensor1 = Pin(SENSOR_PIN1, Pin.IN)
sensor2 = Pin(SENSOR_PIN2, Pin.IN)
sensor3 = Pin(SENSOR_PIN3, Pin.IN)  

last_state = None

class Motor:
    def __init__(self, dirPin, PWMPin):
        self.mDir = Pin(dirPin, Pin.OUT)  # set motor direction pin
        self.pwm = PWM(Pin(PWMPin))  # set motor pwm pin
        self.pwm.freq(1000)  # set PWM frequency
        self.pwm.duty_u16(0)  # set duty cycle - 0=off
        
    def off(self):
        self.pwm.duty_u16(0)
        
    def Forward(self, speed):
        self.mDir.value(0)                     # forward = 0 reverse = 1 motor
        self.pwm.duty_u16(int(65535 * speed / 100))  # speed range 0-100 motor
    def Reverse(self, speed):
        self.mDir.value(1)
        self.pwm.duty_u16(int(65535 * speed / 100))
    def Stop(self):
        self.pwm.duty_u16(0)

def RotateLeft(motorA, motorB, speed, duration):
    motorA.Reverse(speed)
    motorB.Forward(speed)
    sleep(duration)
    motorA.Stop()
    motorB.Stop()

def RotateRight(motorA, motorB, speed, duration):
    motorA.Forward(speed)
    motorB.Reverse(speed)
    sleep(duration)
    motorA.Stop()
    motorB.Stop()

def Shift(motorA, motorB, speed, duration):
    motorA.Forward(speed)
    motorB.Forward(speed)
    sleep(duration)
    motorA.Stop()
    motorB.Stop()

def test_move():
    motorA = Motor(dirPin=4, PWMPin=5)  # Motor 3 is controlled from Motor Driv2 #1, which is on GP4/5
    motorB = Motor(dirPin=7, PWMPin=6)  # Motor 4 is controlled from Motor Driv2 #2, which is on GP6/7



    while True:
        clr1 = sensor1.value()           # 0 for black or 1 for white
        clr2 = sensor2.value()           # 0 for black or 1 for white
        clr3 = sensor3.value()
        last_state1 = None

        # Light LED when black line is detected
        led.value(clr1)

        # Print only on change to keep output tidy
        if clr2 == 0 or clr3 == 0:
            if clr1 != last_state1:
                if clr1 == 0:
                    print("black")
                    motorA.Stop()
                    motorB.Stop()
                else:
                    print("white")
                    motorA.Forward(100)
                    motorB.Forward(100)
                last_state1 = clr1
        elif clr2 == 1 and clr3 == 0:
            Shift(motorA, motorB, 100, 0.5)
            RotateRight(motorA, motorB, 100, 0.3)
            Shift(motorA, motorB, 100, 0.5)
        elif clr2 == 0 and clr3 == 1:
            Shift(motorA, motorB, 100, 0.5)
            RotateLeft(motorA, motorB, 100, 0.3)
            Shift(motorA, motorB, 100, 0.5)
        elif clr2 == 1 and clr3 == 1:
            Shift(motorA, motorB, 100, 0.5)
            RotateLeft(motorA, motorB, 100, 0.3)
            Shift(motorA, motorB, 100, 0.5)

            
            
        # Motor 3 is controlled from Motor Driv2 #1, which is on GP4/5




if __name__ == "__main__":
    test_move()