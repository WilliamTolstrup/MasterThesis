import RPi.GPIO as GPIO
import time

# Pin Definitions:
step_pin = 23  # GPIO pin connected to the "step" pin on the A4988 driver.
dir_pin = 24   # GPIO pin connected to the "DIR" pin on the A4988 driver.
enable_pin = 25  # GPIO pin connected to the "ENABLE" pin on the A4988 driver.
MS1_pin = 5
MS2_pin = 6
MS3_pin = 26
button_pin = 16  # GPIO pin connected to the button

# Setup the GPIO pins
GPIO.setmode(GPIO.BCM)  # Broadcom pin-numbering scheme
GPIO.setup(step_pin, GPIO.OUT)
GPIO.setup(dir_pin, GPIO.OUT)
GPIO.setup(enable_pin, GPIO.OUT)
GPIO.setup(MS1_pin, GPIO.OUT)
GPIO.setup(MS2_pin, GPIO.OUT)
GPIO.setup(MS3_pin, GPIO.OUT)
GPIO.setup(button_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # Set as input with pull-up

motor_direction = GPIO.HIGH  # Initial direction

# Function to control the motor
def rotate_motor(steps, direction):
    GPIO.output(enable_pin, GPIO.LOW)  # Enable the driver
    GPIO.output(MS1_pin, GPIO.LOW)
    GPIO.output(MS2_pin, GPIO.LOW)
    GPIO.output(MS3_pin, GPIO.LOW)    
    GPIO.output(dir_pin, direction)  # Set the direction
    for i in range(steps):           # Loop through the number of steps
        GPIO.output(step_pin, GPIO.HIGH)
        time.sleep(0.001)            # This delay controls the speed
        GPIO.output(step_pin, GPIO.LOW)
        time.sleep(0.001)
    GPIO.output(enable_pin, GPIO.HIGH)  # Disable the driver after operation

def button_callback(channel):
    global motor_direction
    if motor_direction == GPIO.HIGH:
        motor_direction = GPIO.LOW
        rotate_motor(1000, motor_direction)  # Rotate 1000 steps in the toggled direction
    else:
        motor_direction = GPIO.HIGH
        rotate_motor(1000, motor_direction)  # Rotate 1000 steps in the toggled direction

# Add event detection on the button pin
GPIO.add_event_detect(button_pin, GPIO.FALLING, callback=button_callback, bouncetime=300)

try:
    while True:
        time.sleep(0.1)  # Main loop just sleeps as all the action is in the button callback
finally:
    GPIO.cleanup()  # Clean up GPIO on normal exit
