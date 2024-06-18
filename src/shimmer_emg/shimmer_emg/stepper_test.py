import RPi.GPIO as GPIO
import time

# Pin Definitions
button_pin = 16  # GPIO pin connected to the button

# Setup GPIO
GPIO.setmode(GPIO.BCM)  # Broadcom pin-numbering scheme
GPIO.setup(button_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)  # Set as input with pull-down

def button_callback(channel):
    print("Button pressed!")

# Add event detection on button press
GPIO.add_event_detect(button_pin, GPIO.FALLING, callback=button_callback, bouncetime=200)

try:
    print("Press the button to see if it works...")
    while True:
        time.sleep(0.1)  # Sleep to reduce CPU usage
except KeyboardInterrupt:
    print("Exiting program...")

finally:
    GPIO.cleanup()  # Clean up GPIO on exit
