import RPi.GPIO as GPIO
import time

# Pin Definitions:
step_pin = 23  # GPIO pin connected to the "step" pin on the A4988 driver.
dir_pin = 24   # GPIO pin connected to the "DIR" pin on the A4988 driver.
enable_pin = 25  # GPIO pin connected to the "ENABLE" pin on the A4988 driver.

# Setup the GPIO pins
GPIO.setmode(GPIO.BCM)  # Broadcom pin-numbering scheme
GPIO.setup(step_pin, GPIO.OUT)
GPIO.setup(dir_pin, GPIO.OUT)
GPIO.setup(enable_pin, GPIO.OUT)

# Function to control the motor
def rotate_motor(steps, direction):
    GPIO.output(enable_pin, GPIO.LOW)  # Enable the driver
    GPIO.output(dir_pin, direction)  # Set the direction
    for i in range(steps):           # Loop through the number of steps
        GPIO.output(step_pin, GPIO.HIGH)
        time.sleep(0.001)            # This delay controls the speed
        GPIO.output(step_pin, GPIO.LOW)
        time.sleep(0.001)
    GPIO.output(enable_pin, GPIO.HIGH)  # Disable the driver after operation

try:
    # Rotate motor
    rotate_motor(200, GPIO.HIGH)  # Rotate 200 steps clockwise
    time.sleep(2)                 # Wait 2 seconds
    rotate_size=rotate_motor(200, GPIO.LOW) # Rotate 200 steps counterclockwise

finally:
    GPIO.cleanup()  # Clean up GPIO on normal exit
