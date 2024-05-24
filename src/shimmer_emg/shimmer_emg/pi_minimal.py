import RPi.GPIO as GPIO
import time
import keyboard  # Import the keyboard library

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
    pwm_backward.start(0)  # Start PWM with 0% duty cycle (motor off)

    return pwm_forward, pwm_backward

def motor_forward(pwm, speed=50):
    pwm.ChangeDutyCycle(speed)  # Set the speed as duty cycle percentage
    GPIO.output(IN2, GPIO.LOW)  # Ensure backward is off

def motor_backward(pwm, speed=50):
    pwm.ChangeDutyCycle(speed)  # Set the speed as duty cycle percentage
    GPIO.output(IN1, GPIO.LOW)  # Ensure forward is off

def main():
    pwm_forward, pwm_backward = setup()
    try:
        while True:  # Keep the program running
            if keyboard.is_pressed('f'):  # If 'f' is pressed, move forward
                motor_forward(pwm_forward)
                print("Moving forward")
            elif keyboard.is_pressed('b'):  # If 'b' is pressed, move backward
                motor_backward(pwm_backward)
                print("Moving backward")
            elif keyboard.is_pressed('q'):  # If 'q' is pressed, quit the program
                print("Program exiting")
                break
            time.sleep(0.1)  # Short delay to prevent repeated detection of key press
    finally:
        pwm_forward.stop()   # Stop the PWM
        pwm_backward.stop()  # Stop the PWM
        GPIO.cleanup()       # Clean up GPIO settings

if __name__ == '__main__':
    main()
