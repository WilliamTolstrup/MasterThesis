import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
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
                                       'emg_envelope_ch1', 'emg_envelope_ch2'
                                       'ln_acc_x', 'ln_acc_y', 'ln_acc_z',   
                                       'ch1_contraction', 'ch2_contraction', 
                                       'acc_y_derivative',
                                       'elbow_angle',
                                       'state'])

        self.iteration = 1

        # Don't record data for the first 5 seconds to avoid noise
        self.initial_delay_duration = 5  # seconds
        self.delay_end_time = time.time() + self.initial_delay_duration  # Current time + delay

        # Initialize lists and flags
        self.timestamp_data = [0, 0]
        self.emg_raw_data = [0, 0]
        self.emg_filtered_data  = [0, 0]
        self.emg_rectified_data = [0, 0]
        self.emg_envelope_data = [0, 0]
        self.ln_acc_data  = [0, 0, 0]
        self.ch1_contraction_data = [False]
        self.ch2_contraction_data = [False]
        self.derivative_data = [0]
        self.angle_data = [0]
        self.current_state = 'rest'
        # Flags
        self.new_timestamp_data = False
        self.new_emg_raw_data = False
        self.new_emg_filtered_data = False
        self.new_emg_rectified_data = False
        self.new_emg_envelope_data = False
        self.new_ln_acc_data = False
        self.new_ch1_contraction_data = False
        self.new_ch2_contraction_data = False
        self.new_derivative_data = False
        self.new_angle_data = False


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
        self.emg_ch1_contraction_subscriber = self.create_subscription(Bool, '/features/ch1_contractions', self.ch1_contraction_callback, 10)
        self.emg_ch2_contraction_subscriber = self.create_subscription(Bool, '/features/ch2_contractions', self.ch2_contraction_callback, 10)
        # Acc derivative data subscriber
        self.acc_derivative_subscriber = self.create_subscription(Float32, '/features/AccDerivative', self.acc_derivative_callback, 10)
        # Angle estimate
        self.angle_estimate_subscriber = self.create_subscription(Float32, '/features/angle', self.angle_callback, 10)


        # Timing and protocol management
        self.start_time = time.time()
        self.current_state = 'rest'
        self.state_start_time = self.start_time
        self.protocol = [
            ('rest_heavy_vertical', 4),      #Change between "heavy - light", and "vertical - horizontal", to get a decent dataset
            ('flexion_heavy_vertical', 4),
            ('extension_heavy_vertical', 4),
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
        self.emg_envelope_data = [msg.x, msg.y]
        self.new_emg_envelope_data = True
        self.log_and_save_data()

    def ln_acc_callback(self, msg):
        self.ln_acc_data = [msg.x, msg.y, msg.z]
        self.new_ln_acc_data = True
        self.log_and_save_data()

    def ch1_contraction_callback(self, msg):
        self.ch1_contraction_data = msg.data
        self.new_ch1_contraction_data = True
        self.log_and_save_data()

    def ch2_contraction_callback(self, msg):
        self.ch2_contraction_data = msg.data
        self.new_ch2_contraction_data = True
        self.log_and_save_data()

    def acc_derivative_callback(self, msg):
        self.derivative_data = msg.data
        self.new_derivative_data = True
        self.log_and_save_data()

    def angle_callback(self, msg):
        self.angle_data = msg.data
        self.new_angle_data = True
        self.log_and_save_data()


    def update_protocol_state(self):
        current_time = time.time()
        elapsed_time = current_time - self.state_start_time
        _, duration = self.protocol[self.protocol_index]  # Get current state duration

        if elapsed_time >= duration:
            # Move to the next state in the protocol
            self.protocol_index = (self.protocol_index + 1) % len(self.protocol)
            self.current_state, _ = self.protocol[self.protocol_index]  # Update to new state
            self.state_start_time = current_time  # Reset state start time
            print(f"Switching to {self.current_state}.")
            if self.current_state == 'rest_heavy_vertical' or self.current_state == 'rest_light_vertical' or self.current_state == 'rest_heavy_horizontal' or self.current_state == 'rest_light_horizontal':
                print(self.iteration)
                self.iteration += 1

                if self.iteration == 51:
                    print("=========== STOP ===========")

    def log_and_save_data(self):
        if time.time() < self.delay_end_time:
            return  # Skip logging and saving data during the delay period
        
        # Check if all data types have been updated
        if self.new_timestamp_data and self.new_emg_raw_data and self.new_emg_filtered_data and self.new_emg_rectified_data and self.new_emg_envelope_data and self.new_ln_acc_data and self.new_ch1_contraction_data and self.new_ch2_contraction_data and self.new_derivative_data and self.new_angle_data:
            # Save data to csv
            self.data_writer.writerow([self.timestamp_data] + self.emg_raw_data + self.emg_filtered_data + self.emg_rectified_data + self.emg_envelope_data + self.ln_acc_data + self.ch1_contraction_data + self.ch2_contraction_data + self.derivative_data + self.angle_data + [self.current_state])

            self.new_timestamp_data = False
            self.new_emg_raw_data = False
            self.new_emg_filtered_data = False
            self.new_emg_rectified_data = False
            self.new_emg_envelope_data = False
            self.new_ln_acc_data = False
            self.new_ch1_contraction_data = False
            self.new_ch2_contraction_data = False
            self.new_derivative_data = False
            self.new_angle_data = False

        else:
            return
        
    def __del__(self):
        # Close the file
        self.data_file.close()


def main(args=None):
    rclpy.init(args=args)

    data_listener = DataListener()

    rclpy.spin(data_listener)

    data_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
