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

def motor_control(direction, duration):
    if direction == "forward":
        GPIO.output(IN1, GPIO.HIGH)
        GPIO.output(IN2, GPIO.LOW)
    elif direction == "backward":
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.HIGH)
    
    time.sleep(duration)
    
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)

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
        motor_control("forward", 1)
        for _ in range(100):  # Read encoder 100 times in the 1 second period
            read_encoder()
        motor_control("backward", 1)
        for _ in range(100):  # Read encoder 100 times in the 1 second period
            read_encoder()

except KeyboardInterrupt:
    GPIO.cleanup()
