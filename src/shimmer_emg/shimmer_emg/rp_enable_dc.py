import RPi.GPIO as GPIO
import time
import joblib
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np

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

# Load SVM model and scaler
svm_model = joblib.load('/home/pi/MasterThesis/src/shimmer_emg/shimmer_emg/svm_model.pk1')
scaler = joblib.load('/home/pi/MasterThesis/src/shimmer_emg/shimmer_emg/scaler.pk1')

# TODO: Get this working again
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

    def listener_callback(self, feature_msg):
        features = feature_msg.data
        features = np.array(features).reshape(1,-1)
        scaled_features = scaler.transform(features)
        new_state = svm_model.predict(scaled_features)[0]

        print("OG features")
        print(features)
        print("Scaled features")
        print(scaled_features)
        self.signal_strength = 10 # Debug
#        self.signal = (features[0] + features[1]) / 2  # Raw MAV EMG ch1 and ch2   #TODO: Testing if this is fine, of if I should subscribe to raw/filtered emg, too, and use that
        #self.signal = (features[9] + features[10]) / 2 # Filtered MAV EMG ch1 and ch2

        # Thresholds are stand-ins right now, as I haven't tested regular signal strengths.

 #       if self.signal <= 10:
 #           self.signal_strength = 100
 #       elif self.signal <= 30:
 #           self.signal_strength = 75
 #       elif self.signal <= 50:
 #           self.signal_strength = 50
 #       elif self.signal <= 70:
 #           self.signal_strength = 25
 #       elif self.signal > 70:
 #           self.signal_strength = 10 # TODO: Not sure if should be 0, need to test!!! 

        print("Predicted state: ")
        print(self.current_state)

        if new_state != self.current_state:
            self.current_state = new_state
            self.control_motor(self.current_state, self.signal_strength)

    def control_motor(self, state, signal_strength):
        # Adjust speed based on signal strength
        speed = max(0, min(100, signal_strength))

        if state == 'flexion_heavy_vertical' or state == 'flexion_light_vertical' or state == 'flexion_heavy_horizontal' or state == 'flexion_light_horizontal':
            motor_control("forward", speed)
        elif state == 'extension_heavy_vertical' or state == 'extension_light_vertical' or state == 'extension_heavy_horizontal' or state == 'extension_light_horizontal':
            motor_control("backward", speed)
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
