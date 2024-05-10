#!/usr/bin/env python3

from . import shimmer
from . import util
from geometry_msgs.msg import Vector3
from scipy.signal import butter, lfilter, lfilter_zi, iirnotch
import rclpy
from rclpy.node import Node
import time
import numpy as np
from std_msgs.msg import Float32, Bool, Float32MultiArray
import pandas as pd
import os
from collections import deque

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
        self.pubContraction = self.create_publisher(Vector3, '/features/contractions', 10)
        self.pubAccDerivative = self.create_publisher(Float32, '/features/AccDerivative', 10)
        self.pubAngle = self.create_publisher(Float32, '/features/angle', 10)

        self.pubFeatures = self.create_publisher(Float32MultiArray, '/features/feature_list', 10)


        # Individual calibration
        self.data_filepath = '/home/william/repos/control_system_ws/src/shimmer_emg/shimmer_emg/calibration.csv'
        self.rp_filepath = '/home/pi/MasterThesis/src/shimmer_emg/shimmer_emg/calibration.csv' # used on the raspberry pi
        self.data_file_exists = os.path.isfile(self.data_filepath) or os.path.isfile(self.rp_filepath)

        # Flags so it works without calibration (to calibrate)
        self.ch1_MinVC   = False
        self.ch1_MVC     = False
        self.ch2_MinVC   = False
        self.ch2_MVC     = False
        self.accel_y_max = False
        self.accel_y_min = False

        if self.data_file_exists:
            individual_calibration = pd.read_csv(self.data_filepath)
            self.accel_y_max = individual_calibration['max_acc_y'].values
            self.accel_y_min = individual_calibration['min_acc_y'].values
            self.ch1_MVC = individual_calibration['max_mvc_ch1'].values
            self.ch2_MVC = individual_calibration['max_mvc_ch2'].values
            self.ch1_MinVC = individual_calibration['minvc_ch1'].values
            self.ch2_MinVC = individual_calibration['minvc_ch2'].values
            print()
            print("==================================")
            print("Individual calibration: ")
            print(f"Max Accel Y: {self.accel_y_max}")
            print(f"Min Accel Y: {self.accel_y_min}")
            print(f"Ch1 MVC: {self.ch1_MVC}")
            print(f"Ch1 MVC: {self.ch2_MVC}")
            print(f"Ch1 MinVC: {self.ch1_MinVC}")
            print(f"Ch2 MinVC: {self.ch2_MinVC}")
            print("==================================")
            print()
        else:
            print("No individual calibration available")

        self.ch1_threshold = self.setup_envelope_threshold(self.ch1_MinVC, self.ch1_MVC, 0.12) # k = 0.12 -> 12% above minvc towards mvc
        self.ch2_threshold = self.setup_envelope_threshold(self.ch2_MinVC, self.ch2_MVC, 0.08) # Same here

        # Window segmentation related
        self.Fs = 500 # Sampling rate
        self.nyq = self.Fs/2 # Nyquist frequency

        # Wait 5 seconds before record command
        self.countdown = 5

        self.shimmer3 = self.setup_shimmer() # Connecting to shimmer, choosing sensors, etc, etc.
        self.setup_filters()

        self.timer = self.create_timer(0.01, self.sendDataLoop)

    def setup_envelope_threshold(self, minvc, mvc, k):
        if minvc and mvc:
            return minvc + k * (mvc - minvc)
        else:
            return 0


    def setup_shimmer(self):
        TYPE = util.SHIMMER_ExG_0
        PORT = '/dev/rfcomm0'  #####  Bluetooth connection #####

        shimmer3 = shimmer.Shimmer3(TYPE, debug=True)
        shimmer3.connect(com_port=PORT, write_rtc=True, update_all_properties=True, reset_sensors=True)
        shimmer3.set_sampling_rate(500.0)

        shimmer3.set_enabled_sensors(util.SENSOR_ExG1_24BIT, util.SENSOR_LOW_NOISE_ACCELEROMETER)
        gain = util.ExG_GAIN_12  # gain is 12 for EMG
        shimmer3.exg_send_emg_settings(gain)
        shimmer3.print_object_properties()
        
        time.sleep(2)

        shimmer3.start_bt_streaming()

        return shimmer3

    def setup_filters(self):
        # Filter frequency cutoffs and orders. Abreviations: Fco_bp = Filter cutoff bandpass
        Fco_bp = [10, self.Fs * 0.9]  # Bandpass cutoff: 10 Hz and 450 Hz
        Fco_bs = [47, 53]             # Bandstop cutoffs for line noise removal (usually 50 Hz) # Acts as the notch filter
        Fco_lp = 350                  # Lowpass cutoff for noise removal above 350 Hz
        order_bp = 10                 # Order for bandpass filter
        order_bs = 6                  # Order for bandstop filter
        order_lp = 4                  # Order for lowpass filter

        # Normalized cutoff frequencies for filters
        normal_cutoff_bp = [f / (self.Fs) for f in Fco_bp]  # Normalize for bandpass
        normal_cutoff_bs = [f / (self.Fs) for f in Fco_bs]  # Normalize for bandstop
        normal_cutoff_lp = Fco_lp / (self.Fs)               # Normalize for lowpass
    
        # Create filter coefficients for each filter type
        self.b_bp, self.a_bp = butter(order_bp, normal_cutoff_bp, btype='bandpass', analog=False)
        self.b_bs, self.a_bs = butter(order_bs, normal_cutoff_bs, btype='bandstop', analog=False)
        self.b_lp, self.a_lp = butter(order_lp, normal_cutoff_lp, btype='lowpass', analog=False)

        # Initialize filter initial conditions for channel 1
        self.zf_bp_ch1 = lfilter_zi(self.b_bp, self.a_bp)
        self.zf_bs_ch1 = lfilter_zi(self.b_bs, self.a_bs)
        self.zf_lp_ch1 = lfilter_zi(self.b_lp, self.a_lp)

        # Copy initial conditions to channel 2
        self.zf_bp_ch2 = self.zf_bp_ch1.copy()
        self.zf_bs_ch2 = self.zf_bs_ch1.copy()
        self.zf_lp_ch2 = self.zf_lp_ch1.copy()

    def sendDataLoop(self):
        try:
            timestampMsg = Vector3()
            emg_raw_msg      = Vector3()
            emg_filtered_msg = Vector3()
            emg_rectified_msg = Vector3()
            emg_envelope_msg = Vector3()
            emg_contraction_msg = Vector3()
            ln_accMsg = Vector3()
            acc_derivative_msg = Float32()
            angle_estimate_msg = Float32()
            features_msg = Float32MultiArray()

            accelerometer = AccelerometerMethods(n=5)
            emg_ch1 = EMGMethods(window_size=50, Fs=self.Fs)
            emg_ch2 = EMGMethods(window_size=50, Fs=self.Fs)

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

                    # Assign unfiltered values
                    emg_raw_msg.x = float(calibrated_ch1)
                    emg_raw_msg.y = float(calibrated_ch2)

                    # Step 1: Bandpass filter
                    bp_ch1, self.zf_bp_ch1 = lfilter(self.b_bp, self.a_bp, [calibrated_ch1], zi=self.zf_bp_ch1)
                    bp_ch2, self.zf_bp_ch2 = lfilter(self.b_bp, self.a_bp, [calibrated_ch2], zi=self.zf_bp_ch2)

                    # Step 2: Bandstop filter
                    bs_ch1, self.zf_bs_ch1 = lfilter(self.b_bs, self.a_bs, bp_ch1, zi=self.zf_bs_ch1)
                    bs_ch2, self.zf_bs_ch2 = lfilter(self.b_bs, self.a_bs, bp_ch2, zi=self.zf_bs_ch2)

                    # Step 3: Lowpass filter
                    lp_ch1, self.zf_lp_ch1 = lfilter(self.b_lp, self.a_lp, bs_ch1, zi=self.zf_lp_ch1)
                    lp_ch2, self.zf_lp_ch2 = lfilter(self.b_lp, self.a_lp, bs_ch2, zi=self.zf_lp_ch2)

                    # Assign filtered values
                    emg_filtered_msg.x = lp_ch1[0]  # lfilter returns a list, take the first element
                    emg_filtered_msg.y = lp_ch2[0]  # Same here

                    ch1_rectified, ch1_envelope = emg_ch1.emg_envelope(lp_ch1[0])
                    ch2_rectified, ch2_envelope = emg_ch2.emg_envelope(lp_ch2[0])
                    combined_envelope = ch1_envelope + ch2_envelope

                    # Check if contraction is above a threshold. Returns True or False
                    ch1_contraction = emg_ch1.detect_activation(ch1_envelope, self.ch1_threshold)
                    ch2_contraction = emg_ch2.detect_activation(ch2_envelope, self.ch2_threshold)
                    combined_contraction = emg_ch1.detect_activation(combined_envelope, self.ch1_threshold)
                    
                    # Assign rectified values
                    emg_rectified_msg.x = ch1_rectified
                    emg_rectified_msg.y = ch2_rectified

                    # Assign envelope
                    if self.ch1_MVC:
                        emg_envelope_msg.x = float((ch1_envelope-self.ch1_MinVC)/(self.ch1_MVC-self.ch1_MinVC))
                        emg_envelope_msg.y = float((ch2_envelope-self.ch2_MinVC)/(self.ch2_MVC-self.ch2_MinVC))
                        emg_envelope_msg.z = np.clip(emg_envelope_msg.x + emg_envelope_msg.y, 0, 1)

                        print(f"Non-normalized envelope ch1: {ch1_envelope}")
                        print(f"Combined envelope: {emg_envelope_msg.z}")
                    else:
                        emg_envelope_msg.x = ch1_envelope
                        emg_envelope_msg.y = ch2_envelope
                        emg_envelope_msg.z = combined_envelope

                    # Assign contractions (feature)
                    emg_contraction_msg.x = float(ch1_contraction)
                    emg_contraction_msg.y = float(ch2_contraction)
                    emg_contraction_msg.z = float(combined_contraction)

                    ###################################
                    ###        Accelerometer        ###
                    ###################################

                    acc_y_smooth = accelerometer.smooth_accelerometer(calibrated_ln_accy)

                    # Derivative of Accel y (feature)
                    acc_y_derivative = accelerometer.smooth_derivative(acc_y_smooth, ts_current)

                    # Estimated angle based on accelerometer data (feature)
                    elbow_angle = accelerometer.estimate_angle(acc_y_smooth, min_accel=self.accel_y_min, max_accel=self.accel_y_max)

                    # Assign accel values
                    ln_accMsg.x = float(calibrated_ln_accx)
                    ln_accMsg.y = float(acc_y_smooth)
                    ln_accMsg.z = float(calibrated_ln_accz)

                    acc_derivative_msg.data = acc_y_derivative
                    angle_estimate_msg.data = float(elbow_angle)

                    ###################################
                    ###          Timestamp          ###
                    ###################################

                    # Assign Timestamp
                    timestampMsg.x = ts_start
                    timestampMsg.y = ts_current


                    ###################################
                    ###           Features          ###
                    ###################################

                    features_msg.data = [float(combined_contraction)] + [float(acc_y_derivative)] + [float(elbow_angle)]

                    
                    ###################################
                    ###           Publish           ###
                    ###################################

                    self.pubTimestamp.publish(timestampMsg)
                    self.pubEmgRaw.publish(emg_raw_msg)
                    self.pubEmgFiltered.publish(emg_filtered_msg)
                    self.pubEmgRectified.publish(emg_rectified_msg)
                    self.pubEmgEnvelope.publish(emg_envelope_msg)
                    self.pubLnAcc.publish(ln_accMsg)
                    self.pubContraction.publish(emg_contraction_msg)
                    self.pubAccDerivative.publish(acc_derivative_msg)
                    self.pubAngle.publish(angle_estimate_msg)

                    self.pubFeatures.publish(features_msg)

        except KeyboardInterrupt:
            self.shimmer3.stop_bt_streaming()
            self.shimmer3.disconnect(reset_obj_to_init=True)


class EMGMethods:
    def __init__(self, window_size=25, Fs=500):
        """
        Initialize circular buffer for smoothing
        
        Parameters:
        window_size (int): Number of points for smoothing. Default is 50.
        """

        self.envelope_buffer = deque(maxlen=window_size)
        self.Fs = Fs # Sampling frequency
        self.low_pass_cutoff = 4 # 4 Hz

        self.b, self.a = butter(4, self.low_pass_cutoff / (self.Fs / 2), btype='low')



    def emg_envelope(self, data):
        # Full-wave rectification
        rectified_emg = np.abs(data)

        # Low-pass filter at 4 Hz
        envelope_emg = lfilter(self.b, self.a, [rectified_emg])

        self.envelope_buffer.append(envelope_emg[0])  # Append filtered data to the buffer
        return rectified_emg, self.get_smoothed_envelope()  # Compute and return the smoothed envelope


    def get_smoothed_envelope(self):
        """Calculate and return the moving average of the buffer contents."""
        if self.envelope_buffer:
            return sum(self.envelope_buffer) / len(self.envelope_buffer)  # Calculate moving average
        return 0  # If buffer is somehow empty, return 0
    

    def detect_activation(self, envelope, threshold):
        """
        Detect if the EMG envelope indicates muscle activation above a specified threshold.
        
        Parameters:
        envelope (float): Current envelope value.
        threshold (float): Activation threshold. Threshold is xxx of MVC for each muscle TODO: Find decent value for threshold
        
        Returns:
        float32: 1 if the envelope exceeds the threshold, 0 otherwise.
        """
        if envelope >= threshold:
            contraction = 1
        elif envelope < threshold:
            contraction = 0

        return contraction
    

class AccelerometerMethods:
    def __init__(self, n=50):
        """
        Initialize the class with a circular buffer for smoothing.
        
        Parameters:
        n (int): Number of points for smoothing. Default is 4.
        """
        self.previous_value = None
        self.previous_time = None
        self.derivatives = deque(maxlen=n)
        self.accelerometer_buffer = deque(maxlen=50)

    def smooth_accelerometer(self, data):
        """
        Smooth the raw accelerometer data
        
        Parameters:
        data (float): Current sensor reading
        
        Returns:
        float: The mean value over the 50 samples"""

        self.accelerometer_buffer.append(data)
        if self.accelerometer_buffer:
            return sum(self.accelerometer_buffer) / len(self.accelerometer_buffer)
        return 0 # In case there is no buffer, return 0

    def calculate_derivative(self, current_value, previous_value, current_time, previous_time):
        """
        Calculate the derivative of a signal.
        
        Parameters:
        current_value (float): Current sensor reading.
        previous_value (float): Previous sensor reading.
        current_time (float): Current timestamp.
        previous_time (float): Previous timestamp.
        
        Returns:
        float: The derivative of the signal, or None if time difference is zero or input is invalid.
        """
        if previous_value is None or previous_time is None:
            # No previous data to compare with
            return float(0)

        delta_value = current_value - previous_value
        delta_time = current_time - previous_time

        if delta_time > 0:
            return float(delta_value / delta_time)
        else:
            # Avoid division by zero
            return None

    def smooth_derivative(self, current_value, current_time):
        """
        Compute the smoothed derivative using moving average.
        
        Parameters:
        current_value (float): Current sensor reading.
        current_time (float): Current timestamp.
        
        Returns:
        float: Smoothed derivative of the signal.
        """
        # Calculate the raw derivative
        derivative = self.calculate_derivative(current_value, self.previous_value, current_time, self.previous_time)

        # Update the previous values
        self.previous_value = current_value
        self.previous_time = current_time

        # Add the current derivative to the circular buffer if valid
        if derivative is not None:
            self.derivatives.append(derivative)

        # Calculate and return the moving average of the derivatives
        return sum(self.derivatives) / len(self.derivatives) if self.derivatives else float(0)
    
    def estimate_angle(self, accel_y, min_accel, max_accel, min_angle=0, max_angle=150):
        """
        Estimate the angle based on accelerometer y-channel data using linear interpolation.

        Parameters:
        accel_y (float): Current accelerometer y-channel reading.
        min_accel (float): Minimum accelerometer y value corresponding to the minimum angle.
        max_accel (float): Maximum accelerometer y value corresponding to the maximum angle.
        min_angle (float): Minimum angle, typically representing full extension.             Default  0   degrees
        max_angle (float): Maximum angle, typically representing full flexion.               Default  150 degrees

        Returns:
        float: Estimated angle in degrees.
        """
        if max_accel:
            #calculate slope, and intercept point
            a = (min_angle - max_angle)/(max_accel - min_accel)

            b = max_angle - a * min_accel

            angle = a * accel_y + b

            # Calculate the proportion of the way accel_y is between min_accel and max_accel
            #proportion = (accel_y - min_accel) / (max_accel - min_accel)

            # Interpolate this proportion linearly between min_angle and max_angle
            #angle = min_angle + proportion * (max_angle - min_angle)
            return angle
        else:
            return 0


def main(args=None):
    rclpy.init(args=args)
    shimmer_node = ShimmerDataNode()
    rclpy.spin(shimmer_node)
    shimmer_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()