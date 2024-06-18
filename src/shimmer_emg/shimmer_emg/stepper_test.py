import RPi.GPIO as GPIO
import time

# Pin Definitions
step_pin = 23    # GPIO pin connected to the "step" pin on the A4988 driver
dir_pin = 24     # GPIO pin connected to the "DIR" pin on the A4988 driver
enable_pin = 25  # GPIO pin connected to the "ENABLE" pin on the A4988 driver
MS1_pin = 5
MS2_pin = 6
MS3_pin = 26
button_pin = 16  # GPIO pin connected to the button

# Step count
steps = 1000

# Initial direction
current_direction = GPIO.HIGH  # Start with moving forward

# Setup GPIO
GPIO.setmode(GPIO.BCM)  # Broadcom pin-numbering scheme
GPIO.setup(step_pin, GPIO.OUT)
GPIO.setup(dir_pin, GPIO.OUT)
GPIO.setup(enable_pin, GPIO.OUT)
GPIO.setup(MS1_pin, GPIO.OUT)
GPIO.setup(MS2_pin, GPIO.OUT)
GPIO.setup(MS3_pin, GPIO.OUT)
GPIO.setup(button_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # Set as input with pull-up

# Disable the motor driver to start with
GPIO.output(enable_pin, GPIO.HIGH)

def move_stepper(direction, steps):
    # Set the direction
    GPIO.output(dir_pin, direction)
    # Enable the motor driver
    GPIO.output(enable_pin, GPIO.LOW)
    
    # Pulse the step pin
    for _ in range(steps):
        GPIO.output(step_pin, GPIO.HIGH)
        time.sleep(0.001)  # Adjust the delay for your stepper driver
        GPIO.output(step_pin, GPIO.LOW)
        time.sleep(0.001)
    
    # Disable the motor driver
    GPIO.output(enable_pin, GPIO.HIGH)

def button_callback(channel):
    global current_direction
    # Move the stepper motor in the current direction
    move_stepper(current_direction, steps)
    
    # Toggle the direction for the next button press
    current_direction = GPIO.LOW if current_direction == GPIO.HIGH else GPIO.HIGH

# Add event detection on button press
GPIO.add_event_detect(button_pin, GPIO.FALLING, callback=button_callback, bouncetime=200)

try:
    print("Press the button to move the stepper motor...")
    while True:
        time.sleep(0.1)  # Sleep to reduce CPU usage
except KeyboardInterrupt:
    print("Exiting program...")

finally:
    GPIO.cleanup()  # Clean up GPIO on exit
