import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from builtin_interfaces.msg import Time
import csv
import os

class DataListener(Node):
    def __init__(self):
        super().__init__('data_listener')
        self.filename = 'data_file.csv'
        self.file_exists = os.path.isfile(self.filename)
        # Open the file in append mode and create a csv writer object
        self.file = open(self.filename, mode='w', newline='')  # 'a' appends new data onto the existing file. 'w' wipes the file before writing.
        self.writer = csv.writer(self.file)
        # Header for .csv file
        if not self.file_exists:
            self.writer.writerow(['timestamp', 'emg_ch1', 'emg_ch2', 'acc_x', 'acc_y', 'acc_z', 'gyro_x', 'gyro_y', 'gyro_z', 'mag_x', 'mag_y', 'mag_z'])

        # Initialize lists and flags
        self.timestamp_data = [0, 0]
        self.emg_data  = [0, 0]
        self.acc_data  = [0, 0, 0]
        self.gyro_data = [0, 0, 0]
        self.mag_data  = [0, 0, 0]
        # Flags
        self.new_timestamp_data = False
        self.new_emg_data = False
        self.new_acc_data = False
        self.new_gyro_data = False
        self.new_mag_data = False


        # Timestamp subscriber
        self.timestamp_subscriber = self.create_subscription(Time, '/shimmer/timestamp', self.timestamp_callback, 10)
        # EMG data subscriber
        self.emg_subscriber = self.create_subscription(Vector3, '/emg/emg_filtered', self.emg_callback, 10)
        # IMU accelerometer data subscriber
        self.acc_subscriber = self.create_subscription(Vector3, '/imu/acc', self.acc_callback, 10)
        # IMU gyroscope data subscriber
        self.gyro_subscriber = self.create_subscription(Vector3, '/imu/gyro', self.gyro_callback, 10)
        # IMU magnetometer data subscriber
        self.mag_subscriber = self.create_subscription(Vector3, '/imu/mag', self.mag_callback, 10)

    def timestamp_callback(self, msg):
        #self.get_logger().info(f'Timestamp: {msg}')
        self.timestamp_data = msg.sec + (msg.nanosec / 1e9)

        #self.timestamp_data = [msg.sec, msg.nanosec]
        self.new_timestamp_data = True
        self.log_and_save_data()

    def emg_callback(self, msg):
        #self.get_logger().info(f'EMG Data: {msg}')
        self.emg_data = [msg.x, msg.y]
        self.new_emg_data = True
        self.log_and_save_data()

    def acc_callback(self, msg):
        #self.get_logger().info(f'Accelerometer Data: {msg}')
        self.acc_data = [msg.x, msg.y, msg.z]
        self.new_acc_data = True
        self.log_and_save_data()

    def gyro_callback(self, msg):
        #self.get_logger().info(f'Gyroscope Data: {msg}')
        self.gyro_data = [msg.x, msg.y, msg.z]
        self.new_gyro_data = True
        self.log_and_save_data()

    def mag_callback(self, msg):
        #self.get_logger().info(f'Magnetometer Data: {msg}')
        self.mag_data = [msg.x, msg.y, msg.z]
        self.new_mag_data = True
        self.log_and_save_data()

    def log_and_save_data(self):
        # Check if all data types have been updated
        if self.new_timestamp_data and self.new_emg_data and self.new_acc_data and self.new_gyro_data and self.new_mag_data:
            # Log data
            self.get_logger().info(f'Timestamp: {self.timestamp_data}, EMG: {self.emg_data}, Acc: {self.acc_data}, Gyro: {self.gyro_data}, Mag: {self.mag_data}')
            # Save data to csv
            self.writer.writerow([self.timestamp_data] + self.emg_data + self.acc_data + self.gyro_data + self.mag_data)


            self.new_timestamp_data = False
            self.new_emg_data = False
            self.new_acc_data = False
            self.new_gyro_data = False
            self.new_mag_data = False
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
