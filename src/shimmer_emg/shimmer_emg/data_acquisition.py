#!/usr/bin/env python3

from . import shimmer
from . import util
from geometry_msgs.msg import Vector3
from scipy.signal import butter, lfilter, lfilter_zi, iirnotch
import rclpy
from rclpy.node import Node
import time
import numpy as np
from std_msgs.msg import Float32, Bool
import pywt
import pandas as pd

class ShimmerDataNode(Node):
    def __init__(self):
        super().__init__('shimmer_data_node')

        # ROS2 Publisher
        # Timestamp
        self.pubTimestamp     = self.create_publisher(Vector3, '/shimmer/timestamp', 10)
        # EMG
        self.pubEmgRaw      = self.create_publisher(Vector3, '/emg/emg_raw', 10)
        self.pubEmgFiltered = self.create_publisher(Vector3, '/emg/emg_filtered', 10)
        self.pubEmgRectified = self.create_publisher(Vector3, '/emg/emg_rectified', 10)
        self.pubEmgEnvelope = self.create_publisher(Vector3, '/emg/emg_envelope', 10)

        # IMU
        self.pubLnAcc       = self.create_publisher(Vector3, '/imu/ln_acc', 10) # Low noise accelerometer

        # Features
        self.pubCh1Contraction = self.create_publisher(Bool, '/features/ch1_contractions', 10)
        self.pubCh2Contraction = self.create_publisher(Bool, '/features/ch2_contractions', 10)
        self.pubAccDerivative = self.create_publisher(Float32, '/features/AccDerivative', 10)
        self.pubAngle = self.create_publisher(Float32, '/features/angle', 10)


        individual_calibration = pd.read_csv('calibration.csv')
        self.accel_y_max = max(individual_calibration['accel_y'].values)
        self.accel_y_min = min(individual_calibration['accel_y'].values)
        self.ch1_MVC = max(individual_calibration['ch1_envelope'].values)
        self.ch2_MVC = max(individual_calibration['ch2_envelope'].values)

        # Window segmentation related
        self.Fs = 500 # Sampling rate
        self.nyq = self.Fs/2 # Nyquist frequency

        # Wait 5 seconds before record command
        self.countdown = 5

        self.shimmer3 = self.setup_shimmer() # Connecting to shimmer, choosing sensors, etc, etc.

        self.timer = self.create_timer(0.01, self.sendDataLoop)

    def setup_shimmer(self):
        TYPE = util.SHIMMER_ExG_0
        PORT = '/dev/rfcomm0'  #####  Bluetooth connection #####

        shimmer3 = shimmer.Shimmer3(TYPE, debug=True)
        shimmer3.connect(com_port=PORT, write_rtc=True, update_all_properties=True, reset_sensors=True)
        shimmer3.set_sampling_rate(500.0)

        shimmer3.set_enabled_sensors(util.SENSOR_ExG1_24BIT, util.SENSOR_LOW_NOISE_ACCELEROMETER)
        gain = util.ExG_GAIN_12  # gain
        shimmer3.exg_send_emg_settings(gain)
        shimmer3.print_object_properties()
        
        time.sleep(2)

        shimmer3.start_bt_streaming()

        return shimmer3

    def notch_filter(self, frequency=50, Q=30):
        w0 = frequency / self.nyq
        b_n, a_n = iirnotch(w0, Q)
        return b_n, a_n

    def butter_filter(self, cutoff, btype, order='2'):
        normal_cutoff = cutoff / self.nyq
        b, a = butter(order, normal_cutoff, btype=btype)
        return b, a
    
    def filter_signal(self, data, b, a):
        zi = lfilter_zi(b,a) * data[0]

        filtered_data, _ = lfilter(b, a, data, zi=zi*data[0])
        return filtered_data
    
    def emg_envelope(self, data):
        # Notch filter at 50 Hz, with 30 quality
        b_notch, a_notch = self.notch_filter()
        filtered_emg = self.filter_signal(data, b_notch, a_notch)

        # High-pass filter at 35 Hz
        b_hp, a_hp = self.butter_filter(35, btype='high')
        filtered_emg = self.filter_signal(filtered_emg, b_hp, a_hp)

        # Full-wave rectification
        rectified_emg = np.abs(filtered_emg)

        # Low-pass filter at 4 Hz
        b_lp, a_lp = self.butter_filter(4, btype='low')
        envelope_emg = self.filter_signal(rectified_emg, b_lp, a_lp)

        return envelope_emg, rectified_emg, filtered_emg

    # Feature functions

    def calculate_derivative(self, current_value, previous_value, current_time, previous_time):
        """
        Calculate the derivative of a signal using current and previous sensor readings along with their timestamps.
        
        Parameters:
        current_value (float): Current sensor reading.
        previous_value (float): Previous sensor reading.
        current_time (float): Current timestamp when the reading was taken.
        previous_time (float): Previous timestamp when the last reading was taken.
        
        Returns:
        float: The derivative of the signal, or None if time difference is zero or input is invalid.
        """
        if previous_value is None or previous_time is None:
            # No previous data to compare with, so derivative cannot be computed
            return None
        
        delta_value = current_value - previous_value
        delta_time = current_time - previous_time
        
        if delta_time > 0:
            return delta_value / delta_time
        else:
            # Avoid division by zero; return None and handle this case appropriately in the calling code
            return None


    def estimate_angle(self, accel_y, min_accel=-6, max_accel=8, min_angle=0, max_angle=150):
        """
        Estimate the angle based on accelerometer y-channel data using linear interpolation.

        Parameters:
        accel_y (float): Current accelerometer y-channel reading.
        min_accel (float): Minimum accelerometer y value corresponding to the minimum angle. Default -6
        max_accel (float): Maximum accelerometer y value corresponding to the maximum angle. Default +8
        min_angle (float): Minimum angle, typically representing full extension.             Default  0   degrees
        max_angle (float): Maximum angle, typically representing full flexion.               Default  150 degrees

        Returns:
        float: Estimated angle in degrees.
        """
        # Calculate the proportion of the way accel_y is between min_accel and max_accel
        proportion = (accel_y - min_accel) / (max_accel - min_accel)

        # Interpolate this proportion linearly between min_angle and max_angle
        angle = min_angle + proportion * (max_angle - min_angle)
        return angle


    def detect_activation(envelope, threshold):
        """
        Detect if the EMG envelope indicates muscle activation above a specified threshold.
        
        Parameters:
        envelope (float): Current envelope value.
        threshold (float): Activation threshold. Threshold is 1/3 of MVC for each muscle
        
        Returns:
        bool: True if the envelope exceeds the threshold, False otherwise.
        """
        return envelope > threshold


    def sendDataLoop(self):
        try:
            timestampMsg = Vector3()
            emg_raw_msg      = Vector3()
            emg_filtered_msg = Vector3()
            emg_rectified_msg = Vector3()
            emg_envelope_msg = Vector3()
            emg_contraction_ch1_msg = Bool()
            emg_contraction_ch2_msg = Bool()
            ln_accMsg = Vector3()
            acc_derivative_msg = Float32()
            angle_estimate_msg = Float32()
            while True:
                n_of_packets, packets = self.shimmer3.read_data_packet_extended(calibrated=True)
                if n_of_packets > 0:
                    raw_data = packets[0]  # Access the inner list

                    #ts, ts_start, ts_current, calibrated_ch1, calibrated_ch2 = raw_data
                    ts, ts_start, ts_current, calibrated_ln_accx, calibrated_ln_accy, calibrated_ln_accz, calibrated_ch1, calibrated_ch2 = raw_data
                    
                    # There is an extra "Status" if using read_emg_acc_packet, or if calibrated=False for _extended. It is unused.

                    #########################################################
                    ###                  DATA PROCESSING                  ###
                    #########################################################

                    ###################################
                    ###             EMG             ###
                    ###################################

                    ch1_envelope, ch1_rectified, ch1_filtered = self.emg_envelope(calibrated_ch1)
                    ch2_envelope, ch2_rectified, ch2_filtered = self.emg_envelope(calibrated_ch2)

                    # Check if contraction is above a threshold. Returns True or False
                    ch1_contraction = self.detect_activation(ch1_envelope, (self.ch1_MVC/3)) # Threshold is 1/3 of maximum contraction
                    ch2_contraction = self.detect_activation(ch2_envelope, (self.ch2_MVC/3))

                    # Assign unfiltered values
                    emg_raw_msg.x = float(calibrated_ch1)
                    emg_raw_msg.y = float(calibrated_ch2)
                    
                    # Assign filtered values
                    emg_filtered_msg.x = ch1_filtered[0]  # lfilter returns a list, take the first element
                    emg_filtered_msg.y = ch2_filtered[0]  # Same here

                    # Assign rectified values
                    emg_rectified_msg.x = ch1_rectified[0]
                    emg_rectified_msg.y = ch2_rectified[0]

                    # Assign envelope
                    emg_envelope_msg.x = ch1_envelope[0]
                    emg_envelope_msg.y = ch2_envelope[0]

                    # Assign contractions (feature)
                    emg_contraction_ch1_msg.data = ch1_contraction
                    emg_contraction_ch2_msg.data = ch2_contraction

                    ###################################
                    ###        Accelerometer        ###
                    ###################################

                    # Derivative of Accel y (feature)
                    acc_y_derivative = self.calculate_derivative(calibrated_ln_accy, previous_ln_accy, ts_current, previous_ts_current)
                    previous_ln_accy = calibrated_ln_accy
                    previous_ts_current = ts_current

                    # Estimated angle based on accelerometer data (feature)
                    elbow_angle = self.estimate_angle(calibrated_ln_accy, min_accel=self.accel_y_min, max_accel=self.accel_y_max)

                    # Assign accel values
                    ln_accMsg.x = float(calibrated_ln_accx)
                    ln_accMsg.y = float(calibrated_ln_accy)
                    ln_accMsg.z = float(calibrated_ln_accz)

                    acc_derivative_msg.data = float(acc_y_derivative)
                    angle_estimate_msg.data = float(elbow_angle)

                    ###################################
                    ###          Timestamp          ###
                    ###################################

                    # Assign Timestamp
                    timestampMsg.x = ts_start
                    timestampMsg.y = ts_current


                    ###################################
                    ###          Countdown          ###
                    ###################################

                    if ts_current - ts_start < 5:
                        print(self.countdown)
                        if self.countdown > 0:
                            self.countdown -= 1
                        else:
                            self.countdown = None

                        if self.countdown == 0:
                            print("   ---   Record!   ---   ")

                    
                    ###################################
                    ###           Publish           ###
                    ###################################

                    self.pubTimestamp.publish(timestampMsg)
                    self.pubEmgRaw.publish(emg_raw_msg)
                    self.pubEmgFiltered.publish(emg_filtered_msg)
                    self.pubEmgRectified.publish(emg_rectified_msg)
                    self.pubEmgEnvelope.publish(emg_envelope_msg)
                    self.pubCh1Contraction.publish(emg_contraction_ch1_msg)
                    self.pubCh2Contraction.publish(emg_contraction_ch2_msg)
                    self.pubLnAcc.publish(ln_accMsg)
                    self.pubAccDerivative.publish(acc_derivative_msg)
                    self.pubAngle.publish(angle_estimate_msg)

        except KeyboardInterrupt:
            self.shimmer3.stop_bt_streaming()
            self.shimmer3.disconnect(reset_obj_to_init=True)


def main(args=None):
    rclpy.init(args=args)
    shimmer_node = ShimmerDataNode()
    rclpy.spin(shimmer_node)
    shimmer_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()