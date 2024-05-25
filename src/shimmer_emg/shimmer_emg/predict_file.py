import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32
from geometry_msgs.msg import Vector3
import joblib
import numpy as np
import pandas as pd
from collections import deque

# Load SVM model and scaler
svm_model = joblib.load('/home/william/repos/control_system_ws/src/shimmer_emg/shimmer_emg/svm_model.pk1')
scaler = joblib.load('/home/william/repos/control_system_ws/src/shimmer_emg/shimmer_emg/scaler.pk1')

class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control_node')
        # Subscribe to feature list topic
        self.subscription = self.create_subscription(Float32MultiArray, '/features/feature_list', self.feature_callback, 1)
        self.angle_estimate_subscriber = self.create_subscription(Float32, '/features/angle', self.angle_callback, 10)
        self.contraction_subscriber = self.create_subscription(Vector3, '/features/contractions', self.contraction_callback, 10)
        # Initialize a deque with a fixed size for smoothing predictions
        self.prediction_buffer = deque(maxlen=10)
        self.angle = 0

    def angle_callback(self, msg):
        self.angle = msg.data
    
    def contraction_callback(self, msg):
        self.ch1_contraction = msg.x
        self.ch2_contraction = msg.y
        self.combined_contraction = msg.z

    def feature_callback(self, msg):
        feature_names = ['combined_contraction', 'emg_envelope_ch1', 'emg_envelope_ch2', 'acc_y_derivative']
        features_df = pd.DataFrame([msg.data], columns=feature_names)

        # Scale features and predict state
        scaled_features = scaler.transform(features_df)
        predicted_state = svm_model.predict(scaled_features)[0]

        # Add the prediction to the buffer
        self.prediction_buffer.append(predicted_state)

        # Compute the most frequent prediction in the buffer
        smoothed_prediction = max(set(self.prediction_buffer), key=self.prediction_buffer.count)

        # 
        #print(f"Prediction: {smoothed_prediction}")
        if smoothed_prediction == 'flexion' and self.ch1_contraction == True:
            print("Flexion")
        elif smoothed_prediction == 'extension' and self.ch2_contraction == True:
            print("Extension")
        else:
            print("Rest")



def main(args=None):
    rclpy.init(args=args)
    node = MotorControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
