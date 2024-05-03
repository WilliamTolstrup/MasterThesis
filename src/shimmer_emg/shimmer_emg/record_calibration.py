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
        self.filename = 'calibration.csv'
        self.file_exists = os.path.isfile(self.filename)

        # Open the file in append mode and create a csv writer object
        self.file = open(self.filename, mode='a', newline='')  # 'a' appends new data onto the existing file. 'w' wipes the file before writing.
        self.writer = csv.writer(self.file)

        # Header for .csv file
        if not self.file_exists:
            self.writer.writerow(['accel_y', 'ch1_envelope', 'ch2_envelope'])



        # Don't record data for the first 5 seconds to avoid noise
        self.initial_delay_duration = 5  # seconds
        self.delay_end_time = time.time() + self.initial_delay_duration  # Current time + delay

        # Initialize lists and flags
        self.emg_envelope_data  = [0, 0]
        self.acc_y_data  = [0]
        # Flags
        self.new_emg_envelope_data = False
        self.new_acc_y_data = False


        # EMG envelope data subscriber
        self.emg_raw_subscriber = self.create_subscription(Vector3, '/emg/emg_envelope', self.emg_envelope_data_callback, 10)
        # IMU accelerometer data subscriber
        self.acc_subscriber = self.create_subscription(Vector3, '/imu/ln_acc', self.acc_callback, 10)


        # Timing and protocol management
        self.start_time = time.time()
        self.current_state = 'rest'
        self.state_start_time = self.start_time
        self.protocol = [
            ('rest', 4),
            ('MVC bicep', 4),
            ('MVC tricep', 4),
        ]
        self.protocol_index = 0
        self.state_transitions = []

        # Log the initial state
        print(f"Starting protocol with {self.protocol[self.protocol_index][0]} for {self.protocol[self.protocol_index][1]} seconds.")


    def emg_envelope_data_callback(self, msg):
        self.emg_envelope_data = [msg.x, msg.y]
        self.new_emg_envelope_data = True
        self.log_and_save_data()


    def acc_callback(self, msg):
        self.acc_data = msg.y
        self.new_acc_data = True
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
        if self.emg_envelope_data and self.new_acc_data:
            # Save data to csv
            self.writer.writerow(self.new_acc_data + self.emg_envelope_data)

            self.emg_envelope_data = False
            self.new_acc_data = False

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
