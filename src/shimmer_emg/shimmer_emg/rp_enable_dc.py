import RPi.GPIO as GPIO
import time

# Motor Driver GPIO Pins
IN1 = 17  # Example GPIO pin
IN2 = 27  # Example GPIO pin

# Encoder GPIO Pins
CLK = 5   # Example GPIO pin
DT = 6    # Example GPIO pin

# Setup
GPIO.setmode(GPIO.BCM)
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(CLK, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(DT, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Global Variables for Encoder
clkLastState = GPIO.input(CLK)
counter = 0

# Initialize PWM for both IN1 and IN2
pwm_forward = GPIO.PWM(IN1, 1000)  # Initialize PWM on IN1 1000Hz frequency
pwm_backward = GPIO.PWM(IN2, 1000)  # Initialize PWM on IN2 1000Hz frequency

# Start PWM with 0% duty cycle (motor off)
pwm_forward.start(0)
pwm_backward.start(0)

def motor_speed(direction, speed, duration):
    # Set motor speed and direction
    if direction == "forward":
        pwm_forward.ChangeDutyCycle(speed)  # Apply PWM to IN1 for forward rotation
        pwm_backward.ChangeDutyCycle(0)  # Ensure IN2 is LOW
    elif direction == "backward":
        pwm_forward.ChangeDutyCycle(0)  # Ensure IN1 is LOW
        pwm_backward.ChangeDutyCycle(speed)  # Apply PWM to IN2 for reverse rotation
    
    # Run motor for the specified duration
    time.sleep(duration)
    
    # Stop motor
    pwm_forward.ChangeDutyCycle(0)  # Stop the motor by setting PWM duty cycle to 0%
    pwm_backward.ChangeDutyCycle(0)  # Stop the motor by setting PWM duty cycle to 0%

def read_encoder():
    global clkLastState
    global counter
    clkState = GPIO.input(CLK)
    dtState = GPIO.input(DT)
    if clkState != clkLastState:
        if dtState != clkState:
            counter += 1
        else:
            counter -= 1
        print("Counter: ", counter)
    clkLastState = clkState
    time.sleep(0.01)  # Debounce time

try:
    while True:
        motor_speed("forward", 25, 0.5)  # Move forward at 50% speed for 1 second
        for _ in range(100):  # Read encoder 100 times in the 1 second period
            read_encoder()
        motor_speed("backward", 25, 0.5)  # Move forward at 50% speed for 1 second
        for _ in range(100):  # Read encoder 100 times in the 1 second period
            read_encoder()

except KeyboardInterrupt:
    GPIO.cleanup()
