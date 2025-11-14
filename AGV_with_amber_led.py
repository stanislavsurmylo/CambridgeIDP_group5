"""
AGV Main Program with Amber LED Integration
Integrates amber LED controller with line-following and motor control
"""
from machine import Pin, PWM
from time import sleep
from amber_led_controller import AmberLEDController, STATE_START_AREA, STATE_MOVING, STATE_LOADING

# Line sensor pins
SENSOR_PIN_BACK = 21
SENSOR_PIN_LEFT = 20
SENSOR_PIN_RIGHT = 19
SENSOR_PIN_CENTER = 18

# Sensor setup
sensor_back = Pin(SENSOR_PIN_BACK, Pin.IN)
sensor_left = Pin(SENSOR_PIN_LEFT, Pin.IN)
sensor_right = Pin(SENSOR_PIN_RIGHT, Pin.IN)
sensor_center = Pin(SENSOR_PIN_CENTER, Pin.IN)

# Motor pins
MOTOR_L_DIR = 4
MOTOR_L_PWM = 5
MOTOR_R_DIR = 7
MOTOR_R_PWM = 6

# Motor speeds
SPEED_FWD = 100
SPEED_TURN = 100
SPEED_PIVOT = 40
SHIFT_MS = 120
CHECK_MS = 10

# Initialize amber LED controller
amber_led = AmberLEDController(control_pin=25)

def is_line(v):
    return v == 1

def is_in_start_area():
    """Detect if AGV is in start area"""
    # Example: Start area has back sensor on line
    return sensor_back.value() == 1

def is_at_loading_station():
    """Detect if AGV is at loading/unloading station"""
    # Implement based on your sensors (distance, color, etc.)
    # For now, placeholder
    return False

class Motor:
    def __init__(self, dirPin, PWMPin):
        self.mDir = Pin(dirPin, Pin.OUT)
        self.pwm = PWM(Pin(PWMPin))
        self.pwm.freq(1000)
        self.pwm.duty_u16(0)

    def off(self):
        self.pwm.duty_u16(0)

    def Forward(self, speed):
        self.mDir.value(0)
        self.pwm.duty_u16(int(65535 * speed / 100))

    def Reverse(self, speed):
        self.mDir.value(1)
        self.pwm.duty_u16(int(65535 * speed / 100))

    def Stop(self):
        self.pwm.duty_u16(0)

def go_forward(mL, mR, speed):
    mL.Forward(speed)
    mR.Forward(speed)

def stop_all(mL, mR):
    mL.Stop()
    mR.Stop()

def agv_main():
    """Main AGV control loop with amber LED state management"""
    motorL = Motor(dirPin=MOTOR_L_DIR, PWMPin=MOTOR_L_PWM)
    motorR = Motor(dirPin=MOTOR_R_DIR, PWMPin=MOTOR_R_PWM)
    
    # Start in start area - LED should be OFF
    amber_led.set_state(STATE_START_AREA)
    
    print("AGV started. Amber LED should be OFF in start area.")
    
    while True:
        # Update LED state based on AGV position/activity
        if is_in_start_area():
            amber_led.set_state(STATE_START_AREA)  # LED OFF
        elif is_at_loading_station():
            amber_led.set_state(STATE_LOADING)  # LED blinks
        else:
            amber_led.set_state(STATE_MOVING)  # LED blinks
        
        # Update LED (blinks if moving/loading, OFF if in start area)
        amber_led.update()
        
        # Read line sensors
        on_back = is_line(sensor_back.value())
        on_left = is_line(sensor_left.value())
        on_right = is_line(sensor_right.value())
        on_center = is_line(sensor_center.value())
        
        # Line following logic (simplified)
        if on_back and on_center:
            go_forward(motorL, motorR, SPEED_FWD)
        elif not on_center and (on_right or on_left):
            if on_right:
                motorL.Forward(SPEED_PIVOT)
                motorR.Stop()
            else:
                motorL.Stop()
                motorR.Forward(SPEED_PIVOT)
        else:
            stop_all(motorL, motorR)
        
        sleep(CHECK_MS / 1000)

if __name__ == "__main__":
    try:
        agv_main()
    except KeyboardInterrupt:
        amber_led.off()
        print("AGV stopped. Amber LED turned off.")

