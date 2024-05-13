import RPi.GPIO as GPIO
import time

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

def main():
    pwm_forward, pwm_backward = setup()
    try:
        motor_forward(pwm_forward, 0.5, 10)  # Motor runs forward at 50% speed for 0.5 seconds
        time.sleep(1)                        # Wait for 1 second
        motor_backward(pwm_backward, 0.5, 10) # Motor runs backward at 50% speed for 0.5 seconds
    finally:
        pwm_forward.stop()   # Stop the PWM
        pwm_backward.stop()  # Stop the PWM
        GPIO.cleanup()       # Clean up GPIO settings

if __name__ == '__main__':
    main()
