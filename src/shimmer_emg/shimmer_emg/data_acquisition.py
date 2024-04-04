#!/usr/bin/env python3

from . import shimmer
from . import util
from geometry_msgs.msg import Vector3
from scipy.signal import butter, lfilter, lfilter_zi
import rclpy
from rclpy.node import Node
import time


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


        self.setup_filters() # EMG filters
        self.shimmer3 = self.setup_shimmer()

        self.timer = self.create_timer(0.01, self.sendDataLoop)

    def setup_shimmer(self):
        TYPE = util.SHIMMER_ExG_1
        PORT = '/dev/rfcomm0'  #####  Bluetooth connection #####

        shimmer3 = shimmer.Shimmer3(TYPE, debug=True)
        shimmer3.connect(com_port=PORT, write_rtc=True, update_all_properties=True, reset_sensors=True)
        shimmer3.set_sampling_rate(500.0)
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

    def sendDataLoop(self):
        try:
            while True:
                n_of_packets, packets = self.shimmer3.read_data_packet_extended(calibrated=True)
                if n_of_packets > 0:

                    raw_data = packets[0]  # Access the inner list

                    # Now, unpack the values from 'data'
                    ts, ts_start, ts_current, calibrated_ln_accx, calibrated_ln_accy, calibrated_ln_accz, calibrated_ch1, calibrated_ch2 = raw_data



                    #########################################################
                    ###                  DATA PROCESSING                  ###
                    #########################################################

                    # Timestamp
                    timestampMsg = Vector3()
                    timestampMsg.x = ts_start
                    timestampMsg.y = ts_current
                    
                    # EMG
                    emg_raw_msg      = Vector3()
                    emg_filtered_msg = Vector3()

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


                    # IMU
                    ln_accMsg = Vector3()
                    wr_accMsg = Vector3()
                    gyroMsg = Vector3()
                    magMsg = Vector3()

                    ln_accMsg.x = float(calibrated_ln_accx)
                    ln_accMsg.y = float(calibrated_ln_accy)
                    ln_accMsg.z = float(calibrated_ln_accz)
                    
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



"""

[281759, 1711539296.7906246, 1711539296.7906246, 2812, 1926, 1723, 45, 75, 23, 7368, 1184, 62176, 65281, 65321, 115, 128, 689671, 689671]
[281887, 1711539296.7906246, 1711539296.7945309, 2813, 1927, 1724, 48, 53, 7,  7364, 1132, 62204, 65281, 65321, 115, 128, 689927, 689927]
[282655, 1711539296.7906246, 1711539296.8179684, 2815, 1926, 1724, 25, 37, 18, 7332, 1176, 62220, 65277, 65329, 119, 128, 689671, 689671]
[283039, 1711539296.7906246, 1711539296.829687,  2812, 1925, 1727, 41, 41, 10, 7336, 1184, 62188, 65281, 65323, 113, 128, 689415, 689415]
[283551, 1711539296.7906246, 1711539296.845312,  2814, 1925, 1726, 28, 25, 31, 7320, 1204, 62204, 65276, 65333, 118, 128, 689415, 689415]
[284063, 1711539296.7906246, 1711539296.860937,  2811, 1927, 1723, 46, 38, 16, 7320, 1180, 62136, 65280, 65326, 118, 128, 689927, 689927]
[284447, 1711539296.7906246, 1711539296.8726559, 2811, 1930, 1720, 64, 45, 26, 7272, 1160, 62144, 65290, 65330, 123, 128, 690695, 690695]
[284959, 1711539296.7906246, 1711539296.8882809, 2811, 1929, 1719, 65, 54, 56, 7308, 1176, 62160, 65288, 65329, 116, 128, 690439, 690439]
[285471, 1711539296.7906246, 1711539296.9039059, 2812, 1927, 1721, 38, 37, 22, 7320, 1168, 62160, 65281, 65328, 115, 128, 689927, 689927]
[285855, 1711539296.7906246, 1711539296.9156246, 2813, 1926, 1721, 46, 17, 17, 7304, 1204, 62188, 65278, 65332, 118, 128, 689671, 689671]
[286623, 1711539296.7906246, 1711539296.939062,  2812, 1920, 1724, 81, 21, 11, 7316, 1276, 62156, 65284, 65327, 115, 128, 688135, 688135]

[359471, 1711539541.7986135, 1711539541.7986135, 3.65, 8.61, 3.90, 91, 9, 65528, 58240, 2680, 63096, 306, 65264, 156, 9.385572201280459, 9.385572201280459]
[264798, 1711543076.5425928, 1711543077.5992334, 1.39, -3.2, 9.48, 32, 73, 17,     2552, 808, 57784, 65458, 65331, 264, 15.293800071136166, 15.293800071136166]





"""