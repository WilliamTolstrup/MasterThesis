import RPi.GPIO as GPIO
import time
import sys  # Import sys to handle command line arguments

# Motor Driver GPIO Pins
IN1 = 17  # GPIO pin for motor forward
IN2 = 27  # GPIO pin for motor backward

def setup():
    GPIO.setmode(GPIO.BCM)  # Set the board mode to Broadcom
    GPIO.setup(IN1, GPIO.OUT)
    GPIO.setup(IN2, GPIO.OUT)

    # Initialize PWM on both IN1 and IN2 with a frequency of 1000 Hz
    pwm_forward = GPIO.PWM(IN1, 1000)
    pwm_backward = GPIO.PWM(IN2, 1000)
    pwm_forward.start(0)  # Start PWM with 0% duty cycle (motor off)
    pwm_backward.start(0) # Start PWM with 0% duty cycle (motor off)

    return pwm_forward, pwm_backward

def motor_forward(pwm, duration, speed):
    pwm.ChangeDutyCycle(speed)  # Set the speed as duty cycle percentage
    GPIO.output(IN2, GPIO.LOW)  # Ensure backward is off
    time.sleep(duration)        # Wait for the duration to pass
    pwm.ChangeDutyCycle(0)      # Turn off the motor

def motor_backward(pwm, duration, speed):
    pwm.ChangeDutyCycle(speed)  # Set the speed as duty cycle percentage
    GPIO.output(IN1, GPIO.LOW)  # Ensure forward is off
    time.sleep(duration)        # Wait for the duration to pass
    pwm.ChangeDutyCycle(0)      # Turn off the motor

def main(direction):
    pwm_forward, pwm_backward = setup()
    try:
        if direction == "forward":
            motor_forward(pwm_forward, 0.5, 50)  # Motor runs forward at 50% speed for 0.5 seconds
        elif direction == "backward":
            motor_backward(pwm_backward, 0.5, 50)  # Motor runs backward at 50% speed for 0.5 seconds
    finally:
        pwm_forward.stop()   # Stop the PWM
        pwm_backward.stop()  # Stop the PWM
        GPIO.cleanup()       # Clean up GPIO settings

if __name__ == '__main__':
    if len(sys.argv) > 1:
        direction = sys.argv[1]  # Get direction from command line argument
        main(direction)
    else:
        print("Usage: python3 script_name.py [forward|backward]")
