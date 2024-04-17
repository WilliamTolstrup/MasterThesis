import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from builtin_interfaces.msg import Time
import csv
import os
import time


class DataListener(Node):
    def __init__(self):
        super().__init__('data_listener')
        self.filename = 'data_file.csv'
        self.file_exists = os.path.isfile(self.filename)
        # Open the file in append mode and create a csv writer object
        self.file = open(self.filename, mode='a', newline='')  # 'a' appends new data onto the existing file. 'w' wipes the file before writing.
        self.writer = csv.writer(self.file)
        # Header for .csv file
        if not self.file_exists:
            self.writer.writerow(['timestamp', 'emg_raw_ch1', 'emg_raw_ch2', 'emg_filtered_ch1', 'emg_filtered_ch2', 'ln_acc_x', 'ln_acc_y', 'ln_acc_z', 'emg_raw_mav_ch1', 'emg_raw_mav_ch2', 'emg_raw_rms_ch1', 'emg_raw_rms_ch2', 'emg_raw_sd_ch1', 'emg_raw_sd_ch2', 'emg_raw_wl_ch1', 'emg_raw_wl_ch2', 'emg_filtered_mav_ch1', 'emg_filtered_mav_ch2', 'emg_filtered_rms_ch1', 'emg_filtered_rms_ch2', 'emg_filtered_sd_ch1', 'emg_filtered_sd_ch2', 'emg_filtered_wl_ch1', 'emg_filtered_wl_ch1', 'acc_mav_x', 'acc_mav_y', 'acc_mav_z', 'state'])

        # Don't record data for the first 5 seconds to avoid noise
        self.initial_delay_duration = 5  # seconds
        self.delay_end_time = time.time() + self.initial_delay_duration  # Current time + delay

        # Initialize lists and flags
        self.timestamp_data = [0, 0]
        self.emg_filtered_data  = [0, 0]
        self.emg_raw_data = [0, 0]
        self.ln_acc_data  = [0, 0, 0]
        self.features_data = []
        self.current_state = 'rest'
        # Flags
        self.new_timestamp_data = False
        self.new_emg_raw_data = False
        self.new_emg_filtered_data = False
        self.new_ln_acc_data = False
        self.new_features_data = False



        # Timestamp subscriber
        self.timestamp_subscriber = self.create_subscription(Vector3, '/shimmer/timestamp', self.timestamp_callback, 10)
        # EMG raw data subscriber
        self.emg_raw_subscriber = self.create_subscription(Vector3, '/emg/emg_raw', self.emg_raw_data_callback, 10)
        # EMG data subscriber
        self.emg_filtered_subscriber = self.create_subscription(Vector3, '/emg/emg_filtered', self.emg_filtered_callback, 10)
        # IMU accelerometer data subscriber
        self.acc_subscriber = self.create_subscription(Vector3, '/imu/ln_acc', self.ln_acc_callback, 10)
        # Features subscriber
        self.features_subscriber = self.create_subscription(Float32MultiArray, '/shimmer/features', self.features_callback, 10)

        # Timing and protocol management
        self.start_time = time.time()
        self.current_state = 'rest'
        self.state_start_time = self.start_time
        self.protocol = [
            ('rest', 4),
            ('flexion', 4), # Maybe add 2 second "other" after flex
            #('static hold', 2),
            ('extension', 4),
            #('static hold', 2),
            #('other', 5)   # If above, make 2 sec here too
        ]
        self.protocol_index = 0
        self.state_transitions = []

        # Log the initial state
        print(f"Starting protocol with {self.protocol[self.protocol_index][0]} for {self.protocol[self.protocol_index][1]} seconds.")



    def timestamp_callback(self, msg):
        #self.get_logger().info(f'Timestamp: {msg}')
        self.timestamp_data = msg.y - msg.x #(current_time - start_time)

        #self.timestamp_data = [msg.sec, msg.nanosec]
        self.new_timestamp_data = True
        self.update_protocol_state()
        self.log_and_save_data()

    def emg_raw_data_callback(self, msg):
        self.emg_raw_data = [msg.x, msg.y]
        self.new_emg_raw_data = True
        self.log_and_save_data()

    def emg_filtered_callback(self, msg):
        #self.get_logger().info(f'EMG Data: {msg}')
        self.emg_filtered_data = [msg.x, msg.y]
        self.new_emg_filtered_data = True
        self.log_and_save_data()

    def ln_acc_callback(self, msg):
        #self.get_logger().info(f'Accelerometer Data: {msg}')
        self.ln_acc_data = [msg.x, msg.y, msg.z]
        self.new_ln_acc_data = True
        self.log_and_save_data()

    def features_callback(self, msg):
        self.features_data = msg.data
        self.new_features_data = True
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


    def log_and_save_data(self):
        if time.time() < self.delay_end_time:
            return  # Skip logging and saving data during the delay period
        
        # Check if all data types have been updated
        if self.new_timestamp_data and self.new_emg_raw_data and self.new_emg_filtered_data and self.new_ln_acc_data and self.new_features_data:
            # Log data
            #self.get_logger().info(f'Timestamp: {self.timestamp_data}, EMG_raw: {self.emg_raw_data}, EMG_filtered: {self.emg_filtered_data}, Acc: {self.acc_data}, Gyro: {self.gyro_data}, Mag: {self.mag_data}')
            # Save data to csv
            self.writer.writerow([self.timestamp_data] + self.emg_raw_data + self.emg_filtered_data + self.ln_acc_data + self.features_data.tolist() + [self.current_state])

            self.new_timestamp_data = False
            self.new_emg_raw_data = False
            self.new_emg_filtered_data = False
            self.new_ln_acc_data = False
            self.new_features_data = False

        else:
            return
        
    def __del__(self):
        # Close the file
        self.file.close()


def main(args=None):
    rclpy.init(args=args)

    data_listener = DataListener()

    rclpy.spin(data_listener)

    data_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
