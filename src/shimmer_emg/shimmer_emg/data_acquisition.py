#!/usr/bin/env python3

from . import shimmer
from . import util
from geometry_msgs.msg import Vector3
from scipy.signal import butter, lfilter, lfilter_zi
import rclpy
from rclpy.node import Node
import time
import numpy as np
from std_msgs.msg import Float32MultiArray

class ShimmerDataNode(Node):
    def __init__(self):
        super().__init__('shimmer_data_node')

        # ROS2 Publisher
        # Timestamp
        self.pubTimestamp     = self.create_publisher(Vector3, '/shimmer/timestamp', 10)
        # EMG
        self.pubEmgRaw      = self.create_publisher(Vector3, '/emg/emg_raw', 10)
        self.pubEmgFiltered = self.create_publisher(Vector3, '/emg/emg_filtered', 10)
        # IMU
        self.pubLnAcc       = self.create_publisher(Vector3, '/imu/ln_acc', 10) # Low noise accelerometer

        # Features
        self.pubFeatures    = self.create_publisher(Float32MultiArray, '/shimmer/features', 10)

        # Window segmentation related
        self.Fs = 500 # Sampling rate
        self.window_size_samples = int(self.Fs * 0.16) # 80 ms window size
        self.window_overlap_samples = int(self.window_size_samples * 0.5) # 50% overlap

        # Init buffers and window counter
        self.emg_raw_buffer_ch1 = []
        self.emg_raw_buffer_ch2 = []
        self.emg_filtered_buffer_ch1 = []
        self.emg_filtered_buffer_ch2 = []
        self.acc_buffer_x = []
        self.acc_buffer_y = []
        self.acc_buffer_z = []
        self.window_counter = 0

        self.setup_filters() # EMG filters
        self.shimmer3 = self.setup_shimmer()

        self.timer = self.create_timer(0.01, self.sendDataLoop)

    def setup_shimmer(self):
        TYPE = util.SHIMMER_ExG_1
        PORT = '/dev/rfcomm0'  #####  Bluetooth connection #####

        shimmer3 = shimmer.Shimmer3(TYPE, debug=True)
        shimmer3.connect(com_port=PORT, write_rtc=True, update_all_properties=True, reset_sensors=True)
        shimmer3.set_sampling_rate(500.0)
        #shimmer3.set_mag_rate(2) # 2 = 50 Hz
        shimmer3.set_enabled_sensors(util.SENSOR_LOW_NOISE_ACCELEROMETER, util.SENSOR_ExG1_24BIT)
        gain = util.ExG_GAIN_12  # gain
        shimmer3.exg_send_emg_settings(gain)
        shimmer3.print_object_properties()
        
        time.sleep(2)

        shimmer3.start_bt_streaming()

        return shimmer3

    def setup_filters(self):
        self.Fs = 1000
        # Bandpass, bandstop, and lowpass filter settings
        Fco_bp = [10, self.Fs * 0.45]
        Fco_bs = [47, 53]
        Fco_lp = 350
        order_bp = 10
        order_lp = 4
        order_bs = 6
        normal_cutoff_bp = [f / (0.5 * self.Fs) for f in Fco_bp]
        normal_cutoff_bs = [f / (0.5 * self.Fs) for f in Fco_bs]
        normal_cutoff_lp = Fco_lp / (0.5 * self.Fs)
        self.b_bp, self.a_bp = butter(order_bp, normal_cutoff_bp, btype='bandpass', analog=False)
        self.b_bs, self.a_bs = butter(order_bs, normal_cutoff_bs, btype='bandstop', analog=False)
        self.b_lp, self.a_lp = butter(order_lp, normal_cutoff_lp, btype='lowpass', analog=False)
        # Initial conditions for filters
        self.zf_bp_ch1 = lfilter_zi(self.b_bp, self.a_bp)
        self.zf_bs_ch1 = lfilter_zi(self.b_bs, self.a_bs)
        self.zf_lp_ch1 = lfilter_zi(self.b_lp, self.a_lp)
        # Copy initial conditions for a second channel
        self.zf_bp_ch2 = self.zf_bp_ch1.copy()
        self.zf_bs_ch2 = self.zf_bs_ch1.copy()
        self.zf_lp_ch2 = self.zf_lp_ch1.copy()

    # Feature functions

    def mav(self, signal):                     # Mean Absolute Value
        return np.mean(np.abs(signal))
    
    def rms(self, signal):                           # Root Mean Square Error
        signal_array = np.array(signal) # numpy array is needed for element-wise operations
        return np.sqrt(np.mean(signal_array**2))
    
    def sd(self, signal):                            # Standard deviation
        return np.std(signal)
    
    def wl(self, signal):                            # Wavelength
        return np.sum(np.abs(np.diff(signal)))

    def sendDataLoop(self):
        try:
            timestampMsg = Vector3()
            emg_raw_msg      = Vector3()
            emg_filtered_msg = Vector3()
            ln_accMsg = Vector3()
            featureMsg = Float32MultiArray()
            while True:
                n_of_packets, packets = self.shimmer3.read_data_packet_extended(calibrated=True)
                if n_of_packets > 0:

                    raw_data = packets[0]  # Access the inner list

                    # Now, unpack the values from 'data'
                    ts, ts_start, ts_current, calibrated_ln_accx, calibrated_ln_accy, calibrated_ln_accz, calibrated_ch1, calibrated_ch2 = raw_data

                    #########################################################
                    ###                  DATA PROCESSING                  ###
                    #########################################################

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

                    # Assign unfiltered values
                    emg_raw_msg.x = float(calibrated_ch1)
                    emg_raw_msg.y = float(calibrated_ch2)

                    # Assign accel values
                    ln_accMsg.x = float(calibrated_ln_accx)
                    ln_accMsg.y = float(calibrated_ln_accy)
                    ln_accMsg.z = float(calibrated_ln_accz)

                    # Timestamp
                    timestampMsg.x = ts_start
                    timestampMsg.y = ts_current

                    #########################################################
                    ###            DATA ACCUMULATION FOR WINDOW           ###
                    #########################################################

                    self.emg_raw_buffer_ch1.append(calibrated_ch1)
                    self.emg_raw_buffer_ch2.append(calibrated_ch2)
                    self.emg_filtered_buffer_ch1.append(lp_ch1)
                    self.emg_filtered_buffer_ch2.append(lp_ch2)
                    self.acc_buffer_x.append(calibrated_ln_accx)
                    self.acc_buffer_y.append(calibrated_ln_accy)
                    self.acc_buffer_z.append(calibrated_ln_accz)

                    self.window_counter += 1

                    if self.window_counter >= self.window_size_samples:
                        # Compute features for the window
                        
                        emg_raw_features      = [self.mav(self.emg_raw_buffer_ch1),      self.mav(self.emg_raw_buffer_ch2),      self.rms(self.emg_raw_buffer_ch1),      self.rms(self.emg_raw_buffer_ch2),      self.sd(self.emg_raw_buffer_ch1),      self.sd(self.emg_raw_buffer_ch2),      self.wl(self.emg_raw_buffer_ch1),      self.wl(self.emg_raw_buffer_ch2)]
                        emg_filtered_features = [self.mav(self.emg_filtered_buffer_ch1), self.mav(self.emg_filtered_buffer_ch2), self.rms(self.emg_filtered_buffer_ch1), self.rms(self.emg_filtered_buffer_ch2), self.sd(self.emg_filtered_buffer_ch1), self.sd(self.emg_filtered_buffer_ch2), self.wl(self.emg_filtered_buffer_ch1), self.wl(self.emg_filtered_buffer_ch2)]
                        imu_features          = [self.mav(self.acc_buffer_x), self.mav(self.acc_buffer_y), self.mav(self.acc_buffer_z)]

                        # print("Features: ")
                        # print(emg_raw_features)
                        # print(emg_filtered_features)
                        # print(imu_features)

                        # Slide the window
                        # self.window_overlap_samples is half the window_size_samples, so we keep half of the window.
                        start_index = self.window_size_samples - self.window_overlap_samples
                        self.emg_raw_buffer_ch1 = self.emg_raw_buffer_ch1[start_index:]
                        self.emg_raw_buffer_ch2 = self.emg_raw_buffer_ch2[start_index:]
                        self.emg_filtered_buffer_ch1 = self.emg_filtered_buffer_ch1[start_index:]
                        self.emg_filtered_buffer_ch2 = self.emg_filtered_buffer_ch2[start_index:]
                        self.acc_buffer_x = self.acc_buffer_x[start_index:]
                        self.acc_buffer_y = self.acc_buffer_y[start_index:]
                        self.acc_buffer_z = self.acc_buffer_z[start_index:]
                        self.window_counter -= self.window_overlap_samples

                        #########################################################
                        ###                  PUBLISH FEATURES                 ###
                        #########################################################

                        # emg_raw_mav_ch1, emg_raw_mav_ch2, emg_raw_rms_ch1, emg_raw_rms_ch2, emg_raw_sd_ch1, emg_raw_sd_ch2, emg_raw_wl_ch1, emg_raw_wl_ch1,    emg_filtered_mav_ch1, emg_filtered_mav_ch2, emg_filtered_rms_ch1, emg_filtered_rms_ch2, emg_filtered_sd_ch1, emg_filtered_sd_ch2, emg_filtered_wl_ch1, emg_filtered_wl_ch1,    acc_mav_x, acc_mav_y, acc_mav_z
                        # Total 19 features (8,8,3)
                        featureMsg.data = emg_raw_features + emg_filtered_features + imu_features
                        self.pubFeatures.publish(featureMsg)

                    
                    # Publish
                    self.pubTimestamp.publish(timestampMsg)
                    self.pubLnAcc.publish(ln_accMsg)
                    self.pubEmgRaw.publish(emg_raw_msg)
                    self.pubEmgFiltered.publish(emg_filtered_msg)

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