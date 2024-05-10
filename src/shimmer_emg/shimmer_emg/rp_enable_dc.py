import RPi.GPIO as GPIO
import time
import joblib
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32
from geometry_msgs.msg import Vector3
import numpy as np
import pandas as pd


# Motor Driver GPIO Pins
IN1 = 17  # Example GPIO pin
IN2 = 27  # Example GPIO pin

CLK = 5
DT = 6

# Load SVM model and scaler
svm_model = joblib.load('/home/pi/MasterThesis/src/shimmer_emg/shimmer_emg/svm_model.pk1')
scaler = joblib.load('/home/pi/MasterThesis/src/shimmer_emg/shimmer_emg/scaler.pk1')

class PIDController:
    def __init__(self, kp, ki, kd, sample_time):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.sample_time = sample_time
        self.clear()

    def clear(self):
        self.setpoint = 0.0
        self.PV = 0.0
        self.last_time = time.time()
        self.integral = 0.0
        self.derivative = 0.0
        self.last_error = 0.0
        self.output = 0.0

    def update(self, setpoint, PV):
        delta_time = time.time() - self.last_time
        if delta_time >= self.sample_time:
            error = setpoint - PV
            self.integral += error * delta_time
            self.derivative = (error - self.last_error) / delta_time
            self.output = (self.kp * error) + (self.ki * self.integral) + (self.kd * self.derivative)
            self.last_error = error
            self.last_time = time.time()
        return self.output

class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control_node')
        self.pid = PIDController(kp=0.1, ki=0.01, kd=0.05, sample_time=0.01)
        self.features_subscriber = self.create_subscription(Float32MultiArray, '/features/feature_list', self.feature_callback, 10)
        self.emg_envelope_subscriber = self.create_subscription(Vector3, '/emg/emg_envelope', self.emg_envelope_callback, 10)
        self.acc_derivative_subscriber = self.create_subscription(Float32, '/features/AccDerivative', self.acc_derivative_callback, 10)
        self.angle_estimate_subscriber = self.create_subscription(Float32, '/features/angle', self.angle_callback, 10)

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(IN1, GPIO.OUT)
        GPIO.setup(IN2, GPIO.OUT)
        GPIO.setup(CLK, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(DT, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        # Init struggle detection
        self.struggle_detected = False
        self.last_emg_envelope = 0
        self.last_derivative = 0
        self.target_speed = 0
        # Safe angle limits
        self.min_angle = 0
        self.max_angle = 150

        # Initialize PWM for both IN1 and IN2
        self.pwm_forward = GPIO.PWM(IN1, 1000)  # Initialize PWM on IN1 1000Hz frequency
        self.pwm_backward = GPIO.PWM(IN2, 1000)  # Initialize PWM on IN2 1000Hz frequency
        self.pwm_forward.start(0)
        self.pwm_backward.start(0)
        self.current_direction = 'stop'

        # Encoder setup
        self.encoder_position = 0
        self.last_clk_state = GPIO.input(CLK)
        GPIO.add_event_detect(CLK, GPIO.BOTH, callback=self.read_encoder)

    def read_encoder(self, channel):
        clk_state = GPIO.input(CLK)
        dt_state = GPIO.input(DT)
        if clk_state != self.last_clk_state:
            if dt_state != clk_state:
                self.encoder_position += 1
            else:
                self.encoder_position -= 1
        self.last_clk_state = clk_state

    def emg_envelope_callback(self, msg):
        average_envelope = (msg.x + msg.y) / 2
        self.last_emg_envelope = average_envelope
        self.evaluate_control_strategy()

    def acc_derivative_callback(self, msg):
        self.last_derivative = msg.data
        self.evaluate_control_strategy()

    def evaluate_control_strategy(self):
        # Adjust motor speed based on the context
        if self.last_emg_envelope > 5e-06 and abs(self.last_derivative) < 1:
            self.struggle_detected = True
        else:
            self.struggle_detected = False

        if self.struggle_detected:
            # Increase support if struggling
            self.target_speed = 50
        else:
            # Normal adaptive speed calculation
            derivative_effect = abs(self.last_derivative) * 10
            print(f"Derivative effect: {derivative_effect}")
            envelope_effect = self.last_emg_envelope
            print(f"Envelope effect: {envelope_effect}")
            self.target_speed = derivative_effect + envelope_effect
            print(f"Target speed pre-norm: {self.target_speed}")
            self.target_speed = np.clip(self.target_speed, 0, 100)
            print(f"Target speed post-norm: {self.target_speed}")

        self.update_motor()

    def angle_callback(self, msg):
        angle = msg.data
        if angle < self.min_angle or angle > self.max_angle:
            self.stop_motor_due_to_safety()

    def stop_motor_due_to_safety(self):
        # Stops the motor if the elbow angle is out of the safe range
        self.pwm_forward.ChangeDutyCycle(0)
        self.pwm_backward.ChangeDutyCycle(0)
        print("Motor stopped because elbow angle exceeded limit")

    def feature_callback(self, feature_msg):
        feature_names = ['combined_contraction', 'acc_y_derivative', 'elbow_angle']
        features_df = pd.DataFrame([feature_msg.data], columns=feature_names)

        scaled_features = scaler.transform(features_df)
        predicted_state = svm_model.predict(scaled_features)[0].strip().strip("'\"")
        print(f"predicted state: {predicted_state}")
        
   # Update motor direction based on SVM state prediction
        if predicted_state == 'flexion_heavy_vertical':
            self.current_direction = 'forward'
        elif predicted_state == 'extension_heavy_vertical':
            self.current_direction = 'backward'
        else:
            self.current_direction = 'stop'
        self.update_motor()

    def measure_current_speed(self):
        # Placeholder - convert encoder counts to actual speed
        return self.encoder_position / time.monotonic()  # Simplified example

    def update_motor(self):
   #     measured_speed = self.measure_current_speed()
   #     pid_output = self.pid.update(self.target_speed, measured_speed)
   #     adjusted_speed = max(0, min(100, abs(pid_output)))

        adjusted_speed = self.target_speed
        print(f"Adjusted speed: {adjusted_speed}")

        # Set motor PWM based on current direction and dynamically adjusted speed
        if self.current_direction == 'forward':
            self.pwm_forward.ChangeDutyCycle(adjusted_speed)
            self.pwm_backward.ChangeDutyCycle(0)
        elif self.current_direction == 'backward':
            self.pwm_forward.ChangeDutyCycle(0)
            self.pwm_backward.ChangeDutyCycle(adjusted_speed)
        else:
            self.pwm_forward.ChangeDutyCycle(0)
            self.pwm_backward.ChangeDutyCycle(0)

def main(args=None):
    rclpy.init(args=args)
    motor_control_node = MotorControlNode()
    rclpy.spin(motor_control_node)
    motor_control_node.destroy_node()
    rclpy.shutdown()
    GPIO.cleanup()

if __name__ == '__main__':
    main()
