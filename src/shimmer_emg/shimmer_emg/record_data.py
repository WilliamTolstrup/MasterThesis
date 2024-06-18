import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray
from geometry_msgs.msg import Vector3
import csv
import os
import time


class DataListener(Node):
    def __init__(self):
        super().__init__('data_listener')
        self.data_filename = 'data_file.csv'
        self.data_file_exists = os.path.isfile(self.data_filename)

        # Open the file in append mode and create a csv writer object
        self.data_file = open(self.data_filename, mode='a', newline='')  # 'a' appends new data onto the existing file. 'w' wipes the file before writing.
        self.data_writer = csv.writer(self.data_file)

        # Header for .csv file
        if not self.data_file_exists:
            self.data_writer.writerow(['timestamp',
                                       'emg_raw_ch1', 'emg_raw_ch2',
                                       'emg_filtered_ch1', 'emg_filtered_ch2',
                                       'emg_rectified_ch1', 'emg_rectified_ch2',
                                       'emg_envelope_ch1', 'emg_envelope_ch2', 'emg_envelope_combined',
                                       'ln_acc_x', 'ln_acc_y', 'ln_acc_z',   
                                       'ch1_contraction', 'ch2_contraction', 'combined_contraction',
                                       'acc_y_derivative',
                                       'elbow_angle',
                                       'state'])


        self.features_filename = 'features.csv'
        self.features_file_exists = os.path.isfile(self.features_filename)

        self.features_file = open(self.features_filename, mode='a', newline='')
        self.features_writer = csv.writer(self.features_file)

        if not self.features_file_exists:
            self.features_writer.writerow(['combined_contraction', 'ch1_envelope', 'ch2_envelope', 'acc_y_smooth', 'acc_y_derivative', 'elbow_angle', 'state'])


        self.delay = 1.0
        self.iteration = 1

        # Don't record data for the first 5 seconds to avoid noise
        self.initial_delay_duration = 4 + self.delay  # seconds
        self.delay_end_time = time.time() + self.initial_delay_duration  # Current time + delay

        # Initialize lists and flags
        self.timestamp_data = [0, 0]
        self.emg_raw_data = [0, 0]
        self.emg_filtered_data  = [0, 0]
        self.emg_rectified_data = [0, 0]
        self.emg_envelope_data = [0, 0, 0] # added z
        self.ln_acc_data  = [0, 0, 0]
        self.contraction_data = [0, 0, 0]
        self.derivative_data = [0]
        self.angle_data = [0]
        self.features_data = []
        self.current_state = 'rest'
        # Flags
        self.new_timestamp_data = False
        self.new_emg_raw_data = False
        self.new_emg_filtered_data = False
        self.new_emg_rectified_data = False
        self.new_emg_envelope_data = False
        self.new_ln_acc_data = False
        self.new_contraction_data = False
        self.new_derivative_data = False
        self.new_angle_data = False
        self.new_features_data = False



        # Timestamp subscriber
        self.timestamp_subscriber = self.create_subscription(Vector3, '/shimmer/timestamp', self.timestamp_callback, 10)
        # EMG raw data subscriber
        self.emg_raw_subscriber = self.create_subscription(Vector3, '/emg/emg_raw', self.emg_raw_data_callback, 10)
        # EMG filtered data subscriber
        self.emg_filtered_subscriber = self.create_subscription(Vector3, '/emg/emg_filtered', self.emg_filtered_callback, 10)
        # EMG rectified data subscriber
        self.emg_rectified_subscriber = self.create_subscription(Vector3, '/emg/emg_rectified', self.emg_rectified_callback, 10)
        # EMG envelope data subscriber
        self.emg_envelope_subscriber = self.create_subscription(Vector3, '/emg/emg_envelope', self.emg_envelope_callback, 10)
        # IMU accelerometer data subscriber
        self.acc_subscriber = self.create_subscription(Vector3, '/imu/ln_acc', self.ln_acc_callback, 10)



        # Contraction data subscriber
        self.emg_contraction_subscriber = self.create_subscription(Vector3, '/features/contractions', self.contraction_callback, 10) # new
        # Acc derivative data subscriber
        self.acc_derivative_subscriber = self.create_subscription(Float32, '/features/AccDerivative', self.acc_derivative_callback, 10)
        # Angle estimate
        self.angle_estimate_subscriber = self.create_subscription(Float32, '/features/angle', self.angle_callback, 10)
        # Features
        self.features_subscriber = self.create_subscription(Float32MultiArray, '/features/feature_list', self.features_callback, 10)


        # Timing and protocol management
        self.start_time = time.time()
        self.current_state = 'rest'
        self.state_start_time = self.start_time
        self.protocol = [
            ('rest', 4),
            ('flexion', 4),
            ('static hold', 4),
            ('hand', 2),
            ('extension', 4),
            ('hand', 2),
        ]
        self.protocol_index = 0
        self.state_transitions = []

        # Log the initial state
        print(f"Starting protocol with {self.protocol[self.protocol_index][0]} for {self.protocol[self.protocol_index][1]} seconds.")


    def timestamp_callback(self, msg):
        self.timestamp_data = msg.y - msg.x #(current_time - start_time)
        self.new_timestamp_data = True
        self.update_protocol_state()
        self.log_and_save_data()

    def emg_raw_data_callback(self, msg):
        self.emg_raw_data = [msg.x, msg.y]
        self.new_emg_raw_data = True
        self.log_and_save_data()

    def emg_filtered_callback(self, msg):
        self.emg_filtered_data = [msg.x, msg.y]
        self.new_emg_filtered_data = True
        self.log_and_save_data()

    def emg_rectified_callback(self, msg):
        self.emg_rectified_data = [msg.x, msg.y]
        self.new_emg_rectified_data = True
        self.log_and_save_data()

    def emg_envelope_callback(self, msg):
        self.emg_envelope_data = [msg.x, msg.y, msg.z]
        self.new_emg_envelope_data = True
        self.log_and_save_data()

    def ln_acc_callback(self, msg):
        self.ln_acc_data = [msg.x, msg.y, msg.z]
        self.new_ln_acc_data = True
        self.log_and_save_data()

    def contraction_callback(self, msg):
        self.contraction_data = [msg.x, msg.y, msg.z]
        self.new_contraction_data = True
        self.log_and_save_data()

    def acc_derivative_callback(self, msg):
        self.derivative_data = msg.data
        self.new_derivative_data = True
        self.log_and_save_data()

    def angle_callback(self, msg):
        self.angle_data = msg.data
        self.new_angle_data = True
        self.log_and_save_data()

    def features_callback(self, msg):
        self.features_data = msg.data
        self.new_features_data = True
        self.log_and_save_features()


    def update_protocol_state(self):
        current_time = time.time()
        elapsed_time = current_time - self.state_start_time - self.delay
        _, duration = self.protocol[self.protocol_index]  # Get current state duration

        if elapsed_time >= duration:
            # Move to the next state in the protocol
            self.protocol_index = (self.protocol_index + 1) % len(self.protocol)
            self.current_state, _ = self.protocol[self.protocol_index]  # Update to new state
            self.state_start_time = current_time  # Reset state start time
            print(f"Switching to {self.current_state}.")
            if self.current_state == 'rest':
                print(self.iteration)
                self.iteration += 1

                if self.iteration == 21:
                    print("=========== STOP ===========")

    def log_and_save_features(self):
        if time.time() < self.delay_end_time:
            return
        
        if self.new_features_data:
            self.features_writer.writerow([self.features_data.tolist() + [self.current_state]])

            self.new_features_data = False

        else:
            return

    def log_and_save_data(self):
        if time.time() < self.delay_end_time:
            return  # Skip logging and saving data during the delay period
        
        # Check if all data types have been updated
        if self.new_timestamp_data and self.new_emg_raw_data and self.new_emg_filtered_data and self.new_emg_rectified_data and self.new_emg_envelope_data and self.new_ln_acc_data and self.new_derivative_data and self.new_angle_data and self.new_contraction_data:
            # Save data to csv
            self.data_writer.writerow([self.timestamp_data] + self.emg_raw_data + self.emg_filtered_data + self.emg_rectified_data + self.emg_envelope_data + self.ln_acc_data + [self.contraction_data] + [self.derivative_data] + [self.angle_data] + [self.current_state])

            self.new_timestamp_data = False
            self.new_emg_raw_data = False
            self.new_emg_filtered_data = False
            self.new_emg_rectified_data = False
            self.new_emg_envelope_data = False
            self.new_ln_acc_data = False
            self.new_contraction_data = False
            self.new_derivative_data = False
            self.new_angle_data = False

        else:
            return
        
    def __del__(self):
        # Close the file
        self.data_file.close()
        self.features_file.close()


def main(args=None):
    rclpy.init(args=args)

    data_listener = DataListener()

    rclpy.spin(data_listener)

    data_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
