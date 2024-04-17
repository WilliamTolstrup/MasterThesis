import RPi.GPIO as GPIO
import time
import joblib
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

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

# Initialize PWM for both IN1 and IN2
pwm_forward = GPIO.PWM(IN1, 1000)  # Initialize PWM on IN1 1000Hz frequency
pwm_backward = GPIO.PWM(IN2, 1000)  # Initialize PWM on IN2 1000Hz frequency

# Start PWM with 0% duty cycle (motor off)
pwm_forward.start(0)
pwm_backward.start(0)

# Global Variables for Encoder
#clkLastState = GPIO.input(CLK)
#counter = 0

def motor_control(action, speed=10):
    if action == "forward":
        pwm_forward.ChangeDutyCycle(speed)
        pwm_backward.ChangeDutyCycle(0)
    elif action == "backward":
        pwm_forward.ChangeDutyCycle(0)
        pwm_backward.ChangeDutyCycle(speed)
    elif action == "stop":
        pwm_forward.ChangeDutyCycle(0)
        pwm_backward.ChangeDutyCycle(0)

# Load SVM model
svm_model = joblib.load('svm_model.pk1')


# def read_encoder():
#     global clkLastState
#     global counter
#     clkState = GPIO.input(CLK)
#     dtState = GPIO.input(DT)
#     if clkState != clkLastState:
#         if dtState != clkState:
#             counter += 1
#         else:
#             counter -= 1
#         print("Counter: ", counter)
#     clkLastState = clkState
#     time.sleep(0.01)  # Debounce time

class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control_node')
        self.subscription = self.create_subscription(Float32MultiArray, '/shimmer/features', self.listener_callback, 10)
        self.current_state = None

    def listener_callback(self, msg):
        features = msg.data
        new_state = svm_model.predict([features])[0]
        if new_state != self.current_state:
            self.current_state = new_state
            self.control_motor(self.current_state)

    def control_motor(self, state):
        if state == 'flexion':
            motor_control("forward")
        elif state == 'extension':
            motor_control("backward")
        else:
            motor_control("stop")

def main(args=None):
    rclpy.init(args=args)
    motor_control_node = MotorControlNode()
    rclpy.spin(motor_control_node)
    motor_control_node.destroy_node()
    rclpy.shutdown()
    GPIO.cleanup()

if __name__ == '__main__':
    main()