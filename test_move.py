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

def RotateLeft(motor1, motor2, speed, duration):
    motor1.Reverse(speed)
    motor2.Forward(speed)
    sleep(duration)
    motor1.Stop()
    motor2.Stop()

def RotateRight(motor1, motor2, speed, duration):
    motor1.Forward(speed)
    motor2.Reverse(speed)
    sleep(duration)
    motor1.Stop()
    motor2.Stop()

def test_move():
    motor1 = Motor(dirPin=4, PWMPin=5)  # Motor 3 is controlled from Motor Driv2 #1, which is on GP4/5
    motor2 = Motor(dirPin=7, PWMPin=6)  # Motor 4 is controlled from Motor Driv2 #2, which is on GP6/7



    while True:
        clr1 = sensor1.value()           # 0 for black or 1 for white
        clr2 = sensor2.value()           # 0 for black or 1 for white
        clr3 = sensor3.value()
        last_state = None

        # Light LED when black line is detected
        led.value(clr1)

        # Print only on change to keep output tidy
        if clr2 == 0 or clr3 == 0:
            if clr1 != last_state:
                if clr1 == 0:
                    print("black")
                    motor1.Stop()
                    motor2.Stop()
                else:
                    print("white")
                    motor1.Forward(100)
                    motor2.Forward(100)
                last_state1 = clr1
        elif clr2 == 1 and clr3 == 0:
            motor1.Forward(100)
            motor2.Forward(100)
            sleep(0.5)
            motor1.Stop()
            motor2.Stop()
            RotateRight(motor1, motor2, 100, 0.3)
        elif clr2 == 0 and clr3 == 1:
            motor1.Forward(100)
            motor2.Forward(100)
            sleep(0.5)
            motor1.Stop()
            motor2.Stop()
            RotateLeft(motor1, motor2, 100, 0.3)
        elif clr2 == 1 and clr3 == 1:
            motor1.Forward(100)
            motor2.Forward(100)
            sleep(0.5)
            motor1.Stop()
            motor2.Stop()
            RotateLeft(motor1, motor2, 100, 0.3)


            
            
        # Motor 3 is controlled from Motor Driv2 #1, which is on GP4/5




if __name__ == "__main__":
    test_move()