import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import Vector3
import csv
import os
import time

class MVCDataListener(Node):
    def __init__(self):
        super().__init__('mvc_data_listener')
        self.data_filename = 'calibration.csv'
        self.data_file = open(self.data_filename, mode='w', newline='')  # Overwrite old data
        self.data_writer = csv.writer(self.data_file)
        self.data_writer.writerow(['max_mvc_ch1', 'max_mvc_ch2', 'minvc_ch1', 'minvc_ch2', 'max_acc_y', 'min_acc_y'])

        # Measurement protocol includes motion states
        self.mvc_measurements = [
            ('prepare', 5),
            ('mvc', 5),
            ('rest', 5),
            ('full range of motion', 10),
            ('rest', 5),
            ('mvc', 5),
            ('rest', 5),
            ('mvc', 5),
            ('finished', 5)
        ]
        self.protocol_index = 0
        self.state_start_time = time.time()

        # Data storage for processing later
        self.emg_envelope_data = [0, 0]
        self.acc_y_data = 0
        self.current_state = 'prepare'
        self.mvc_values = [[], []]
        self.minvc_values = [[], []]
        self.extreme_acc_y = []

        print(f"Startting with state: {self.current_state}")

        # Subscriptions
        self.emg_envelope_subscriber = self.create_subscription(Vector3, '/emg/emg_envelope', self.emg_envelope_callback, 10)
        self.acc_subscriber = self.create_subscription(Vector3, '/imu/ln_acc', self.ln_acc_callback, 10)

    def emg_envelope_callback(self, msg):
        if self.current_state == 'mvc':
            self.mvc_values[0].append(msg.x)
            self.mvc_values[1].append(msg.y)
        
        elif self.current_state == 'rest':
            self.minvc_values[0].append(msg.x)
            self.minvc_values[1].append(msg.y)

    def ln_acc_callback(self, msg):
        if self.current_state == 'full range of motion':
            self.extreme_acc_y.append(msg.y)

    def update_protocol_state(self):
        current_time = time.time()
        elapsed_time = current_time - self.state_start_time
        state, duration = self.mvc_measurements[self.protocol_index]
        
        if elapsed_time >= duration:
            self.protocol_index = (self.protocol_index + 1) % len(self.mvc_measurements)
            self.current_state, _ = self.mvc_measurements[self.protocol_index]
            self.state_start_time = current_time
            print(f"Switching to {self.current_state}.")

            if self.current_state == 'finished':
                # Calculate and log the highest MVC values and extreme AccY values
                max_mvc_ch1 = max(self.mvc_values[0]) if self.mvc_values[0] else 0
                max_mvc_ch2 = max(self.mvc_values[1]) if self.mvc_values[1] else 0
                minvc_ch1 = min(self.minvc_values[0] if self.minvc_values[0] else 0)
                mincv_ch2 = min(self.minvc_values[1] if self.minvc_values[1] else 0)

                max_acc_y = max(self.extreme_acc_y) if self.extreme_acc_y else 0
                min_acc_y = min(self.extreme_acc_y) if self.extreme_acc_y else 0

                # Save summary data to csv
                self.data_writer.writerow([max_mvc_ch1, max_mvc_ch2, minvc_ch1, mincv_ch2, max_acc_y, min_acc_y])
                self.data_file.close()
                rclpy.shutdown()

    def run(self):
        while rclpy.ok():
            rclpy.spin_once(self)
            self.update_protocol_state()

def main(args=None):
    rclpy.init(args=args)
    mvc_data_listener = MVCDataListener()
    mvc_data_listener.run()

if __name__ == '__main__':
    main()
