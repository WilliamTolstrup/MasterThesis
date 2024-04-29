import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Vector3
import csv
import os
import time


class DataListener(Node):
    def __init__(self):
        super().__init__('data_listener')
        self.features_filename = 'features_file.csv'
        self.features_file_exists = os.path.isfile(self.features_filename)

        self.continuous_filename = 'continuous_file.csv'
        self.continuous_file_exists = os.path.isfile(self.continuous_filename)

        # Open the file in append mode and create a csv writer object
        self.features_file = open(self.features_filename, mode='a', newline='')  # 'a' appends new data onto the existing file. 'w' wipes the file before writing.
        self.continuous_file = open(self.continuous_filename, mode='a', newline='')
        self.features_writer = csv.writer(self.features_file)
        self.continuous_writer = csv.writer(self.continuous_file)

        # Header for .csv file
        if not self.features_file_exists:
            self.features_writer.writerow([
                                  'emg_raw_mav_ch1', 'emg_raw_mav_ch2', 
                                  'emg_raw_rms_ch1', 'emg_raw_rms_ch2', 
                                  'emg_raw_sd_ch1', 'emg_raw_sd_ch2', 
                                  'emg_raw_wl_ch1', 'emg_raw_wl_ch2', 
                                  'raw_coeff_1', 'raw_coeff_2', 'raw_coeff_3', 'raw_coeff_4', 'raw_coeff_5', 'raw_coeff_6', 'raw_coeff_7', 'raw_coeff_8', 

                                  'emg_filtered_mav_ch1', 'emg_filtered_mav_ch2', 
                                  'emg_filtered_rms_ch1', 'emg_filtered_rms_ch2', 
                                  'emg_filtered_sd_ch1', 'emg_filtered_sd_ch2', 
                                  'emg_filtered_wl_ch1', 'emg_filtered_wl_ch2', 
                                  'filtered_coeff_1', 'filtered_coeff_2', 'filtered_coeff_3', 'filtered_coeff_4', 'filtered_coeff_5', 'filtered_coeff_6', 'filtered_coeff_7', 'filtered_coeff_8', 

                                  'acc_mav_x', 'acc_mav_y', 'acc_mav_z', 
                                  'acc_rms_x', 'acc_rms_y', 'acc_rms_z',
                                  'acc_sd_x', 'acc_sd_y', 'acc_sd_z',
     #                             'acc_diff_x', 'acc_diff_y', 'acc_diff_z',
                                  'state'])

        if not self.continuous_file_exists:
            self.continuous_writer.writerow(['timestamp',
                                             'emg_raw_ch1', 'emg_raw_ch2',
                                             'emg_filtered_ch1', 'emg_filtered_ch2',
                                             'ln_acc_x', 'ln_acc_y', 'ln_acc_z',
                                             'state'])

        self.iteration = 1

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
        self.timestamp_flag_one = False



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

    def ln_acc_callback(self, msg):
        self.ln_acc_data = [msg.x, msg.y, msg.z]
        self.new_ln_acc_data = True
        self.log_and_save_data()

    def features_callback(self, msg):
        self.features_data = msg.data
        self.new_features_data = True
        self.log_and_save_features()

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
        if self.new_timestamp_data and self.new_emg_raw_data and self.new_emg_filtered_data and self.new_ln_acc_data:
            # Log data
            #self.get_logger().info(f'Timestamp: {self.timestamp_data}, EMG_raw: {self.emg_raw_data}, EMG_filtered: {self.emg_filtered_data}, Acc: {self.acc_data}, Gyro: {self.gyro_data}, Mag: {self.mag_data}')
            # Save data to csv
            self.continuous_writer.writerow([self.timestamp_data] + self.emg_raw_data + self.emg_filtered_data + self.ln_acc_data + [self.current_state])

            self.new_timestamp_data = False
            self.new_emg_raw_data = False
            self.new_emg_filtered_data = False
            self.new_ln_acc_data = False

        else:
            return
        
    def __del__(self):
        # Close the file
        self.features_file.close()
        self.continuous_file.close()


def main(args=None):
    rclpy.init(args=args)

    data_listener = DataListener()

    rclpy.spin(data_listener)

    data_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
