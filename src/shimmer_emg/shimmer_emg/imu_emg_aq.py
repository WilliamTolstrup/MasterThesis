#!/usr/bin/env python3

import sys
import struct
import serial
import signal
import numpy as np
from scipy.signal import butter, lfilter, lfilter_zi
import time
from PyQt5 import QtWidgets, QtCore
from PyQt5.QtCore import pyqtSignal
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Empty, UInt8, UInt64
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Vector3
from collections import deque
from threading import Timer


class Main(QtWidgets.QMainWindow):
    def __init__(self, node):
        super(Main, self).__init__()

        self.node = node  # ROS 2 node
        self.initUI() # GUI
        
        # Define serial object
        self.serialObj = serial.Serial()
        self.serialObj.baudrate = 115200
        self.serialObj.timeout = 1

        # ---------------- Filter setup --------------------
        self.Fs = 1000
        # Bandpass, bandstop, and lowpass filter settings
        Fco_bp = [10, self.Fs * 0.45]
        Fco_bs = [47, 53]
        Fco_lp = 50
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

        # Serial thread and ROS 2 publishers/subscribers
        self.serial_handler = Communicate(self.serialObj, self.node)
        # ------------------- ROS2 Publisher ----------------------
        # Timestamp
        self.pubTimestamp = self.node.create_publisher(Time, '/shimmer/timestamp', 10)
        # EMG
        self.pubEmgRaw = self.node.create_publisher(Vector3, '/emg/emg_raw', 10)
        self.pubEmgFiltered = self.node.create_publisher(Vector3, '/emg/emg_filtered', 10)
        # IMU
        self.pubAcc = self.node.create_publisher(Vector3, '/imu/acc', 10)
        self.pubGyro = self.node.create_publisher(Vector3, '/imu/gyro', 10)
        self.pubMag = self.node.create_publisher(Vector3, '/imu/mag', 10)

        # -------------------- ROS2 Subscription ------------------
        # EMG and IMU
        self.node.create_subscription(Empty, '/shimmer/stop', self.stop_Btn, 10)
        self.node.create_subscription(UInt8, '/shimmer/start', self.start_Btn, 10)


        self.thread1 = QtCore.QThread()
        self.serial_handler.moveToThread(self.thread1)
        self.thread1.started.connect(self.serial_handler.dataSendLoop)
        self.serial_handler.data_signal.connect(self.addData_callbackFunc)

    def initUI(self):
        self.setWindowTitle("EMG Device Controller")
        self.setGeometry(100, 100, 800, 600)

        # Start button
        self.startBtn = QtWidgets.QPushButton("Start", self)
        self.startBtn.move(50, 500)
        self.startBtn.clicked.connect(self.start_Btn)

        # Stop button
        self.stopBtn = QtWidgets.QPushButton("Stop", self)
        self.stopBtn.move(150, 500)
        self.stopBtn.clicked.connect(self.stop_Btn)

        # Status label
        self.statusLabel = QtWidgets.QLabel("Disconnected", self)
        self.statusLabel.move(50, 50)
        self.statusLabel.resize(200, 20)

        # Show the window
        self.show()

    # Remaining methods are updated analogously, adapting PyQt5, Python 3, and ROS 2 usage.

    def start_Btn(self):
        try:
            # Static port number. Remember to change if using a different unit. 5F90.
            port = str('/dev/rfcomm0')
            print(port)
            self.serialObj.port = port
            self.serialObj.open()
            self.serial_handler.reading = True
            self.thread1.start()
            print('####################################')
            print('###  Reading serial port started ###')
            print('####################################')

        except:
            print('This Serial Port Is Not Available')

    def stop_Btn(self,data):
        print("####################################")
        print("### Serial connection terminated ###")
        print("####################################")
        if self.serialObj.is_open:
            self.serial_handler.reading = False
            self.thread1.exit()
            #send stop streaming command
            self.serialObj.write(struct.pack('B', 0x20))
            time.sleep(1)
            self.serial_handler.wait_for_ack() 
            time.sleep(1)
            self.serialObj.close() 
            
    def addData_callbackFunc(self, timestamp_sec, timestamp_nsec, ch1, ch2, accX, accY, accZ, gyroX, gyroY, gyroZ, magX, magY, magZ):
        # Timestamp
        timestampMsg = Time()
        timestampMsg.sec = timestamp_sec
        timestampMsg.nanosec = timestamp_nsec

        # EMG
        # Step 1: Bandpass filter
        bp_ch1, self.zf_bp_ch1 = lfilter(self.b_bp, self.a_bp, [ch1], zi=self.zf_bp_ch1)
        bp_ch2, self.zf_bp_ch2 = lfilter(self.b_bp, self.a_bp, [ch2], zi=self.zf_bp_ch2)

        # Step 2: Bandstop filter
        bs_ch1, self.zf_bs_ch1 = lfilter(self.b_bs, self.a_bs, bp_ch1, zi=self.zf_bs_ch1)
        bs_ch2, self.zf_bs_ch2 = lfilter(self.b_bs, self.a_bs, bp_ch2, zi=self.zf_bs_ch2)

        # Step 3: Lowpass filter
        lp_ch1, self.zf_lp_ch1 = lfilter(self.b_lp, self.a_lp, bs_ch1, zi=self.zf_lp_ch1)
        lp_ch2, self.zf_lp_ch2 = lfilter(self.b_lp, self.a_lp, bs_ch2, zi=self.zf_lp_ch2)

        # Create objects for raw and filtered EMG data
        emg_raw_msg = Vector3()
        emg_filtered_msg = Vector3()

        # Assign raw values
        emg_raw_msg.x = ch1
        emg_raw_msg.y = ch2

        # Assign filtered values
        emg_filtered_msg.x = lp_ch1[0]  # lfilter returns a list, take the first element
        emg_filtered_msg.y = lp_ch2[0]  # Same here

        # IMU
        accMsg = Vector3()
        gyroMsg = Vector3()
        magMsg = Vector3()
        
        accMsg.x = accX
        accMsg.y = accY
        accMsg.z = accZ
        
        gyroMsg.x = gyroX
        gyroMsg.y = gyroY
        gyroMsg.z = gyroZ
        
        magMsg.x = magX
        magMsg.y = magY
        magMsg.z = magZ        
        
        # Publish
        self.pubTimestamp.publish(timestampMsg)
        self.pubAcc.publish(accMsg)
        self.pubGyro.publish(gyroMsg)
        self.pubMag.publish(magMsg)
        self.pubEmgRaw.publish(emg_raw_msg)
        self.pubEmgFiltered.publish(emg_filtered_msg)



class Communicate(QtCore.QObject):
    # A class for configuring the EMG module and reading the data from it
    data_signal = pyqtSignal(int, int, float, float, float, float, float, float, float, float, float, float, float)

    def __init__(self, serialObj, node):
        super(Communicate, self).__init__()
        self.reading = False
        self.serial = serialObj
        self.node = node
        self.buffer = deque()
        self.publish_rate = 0.2 # Seconds, meaining it will publish 5 times per second
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.publish_averaged_data)
        self.timer.start(int(self.publish_rate * 1000)) # milliseconds, also .start required int
        self.last_published_timestamp = 0

    def publish_averaged_data(self):
        if not self.buffer:
            return

        # Calculate average
        average_data = [sum(x) / len(x) for x in zip(*self.buffer)]
        self.buffer.clear()

        # Create timestamp
        current_time = self.node.get_clock().now()
        timestamp_sec = current_time.seconds_nanoseconds()[0]
        timestamp_nsec = current_time.seconds_nanoseconds()[1]
        if timestamp_nsec != self.last_published_timestamp: # ensure that only 1 message is published per timestamp
            self.data_signal.emit(timestamp_sec, timestamp_nsec, *average_data)

    def dataSendLoop(self):
        self.initialize_serial()
        exgGain = {
            'GAIN_1': 1,
            'GAIN_2': 2,
            'GAIN_3': 3,
            'GAIN_4': 4,
            'GAIN_6': 6,
            'GAIN_8': 8,
            'GAIN_12': 12
        }


        exgCalFactor = (((2.42 * 1000) / exgGain['GAIN_12']) / (pow(2, 23) - 1))

        ddata = b""  # Use bytes literal for Python 3
        numbytes = 0
        framesize = 29  # 1 byte packet type + 4 bytes timestamp + 6 bytes emg, 18 bytes imu (6x3 acc, gyro, mag)
        # Sensitivity
        gyro_sensitivity = 131 # Based on MPU-9150 chip data from Invensense +-3 based on temperature, so shouldn't be a problem
        mag_sensitivity_xy = 1100 # Based on LSM303DLHC data from STMicroelectronics. Sensitivity based on +-1.3 field full-scale
        mag_sensitivity_z = 980

        # for the analog accelerometer, we need the ADC the data to convert the data to gs.
        n = 10 # 10-bit ADC (if it doesn't work, try 12)
        adc_resolution = 2**n
        reference_voltage = 3.3
        sensitivity_v_per_g = 0.6 # 600 mV/g
        zero_g_voltage = 1.5 # Zero-g offset is typical 1.5 V with +-0.2 based on temperature

        try:
            while self.reading:
                while numbytes < framesize:
                    ddata += self.serial.read(framesize)
                    numbytes = len(ddata)

                data = ddata[0:framesize]
                ddata = ddata[framesize:]
                numbytes = len(ddata)

                packettype = struct.unpack('B', data[0:1])

                #ts0, ts1, ts2, c1status = struct.unpack('BBBB', data[1:5])
                #timestamp = ts0 + ts1 * 256 + ts2 * 65536

                (accx, accy, accz) = struct.unpack('HHH', data[5:11])
                (gyrox, gyroy, gyroz) = struct.unpack('HHH', data[11:17])
                (magx, magy, magz) = struct.unpack('>hhh', data[17:23])
                ch1 = struct.unpack('>i', data[23:26] + b'\0')[0] >> 8
                ch2 = struct.unpack('>i', data[26:29] + b'\0')[0] >> 8

                # Conversions
                # Convert raw data to voltage
                voltage_x = (accx / adc_resolution) * reference_voltage
                voltage_y = (accy / adc_resolution) * reference_voltage
                voltage_z = (accz / adc_resolution) * reference_voltage

                # Convert voltage to gs
                accx = (voltage_x - zero_g_voltage) / sensitivity_v_per_g
                accy = (voltage_y - zero_g_voltage) / sensitivity_v_per_g
                accz = (voltage_z - zero_g_voltage) / sensitivity_v_per_g

                # Convert to degrees/sec
                gyrox = (gyrox/gyro_sensitivity)
                gyroy = (gyroy/gyro_sensitivity)
                gyroz = (gyroz/gyro_sensitivity)

                # Convert to gs
                magx = (magx / mag_sensitivity_xy)
                magy = (magy / mag_sensitivity_xy)
                magz = (magz / mag_sensitivity_z)

                ch1 *= exgCalFactor
                ch2 *= exgCalFactor

                self.buffer.append([ch1, ch2, accx, accy, accz, gyrox, gyroy, gyroz, magx, magy, magz])
                #self.data_signal.emit(timestamp_sec, timestamp_nsec, ch1, ch2, accx, accy, accz, gyrox, gyroy, gyroz, magx, magy, magz)

        except KeyboardInterrupt:
            self.serial.write(struct.pack('B', 0x20))
            self.wait_for_ack()
            self.serial.close()
            print("All done!")

    def initialize_serial(self):
        samplingFrequency = 1000 # frequency in Hz


        # The internal sampling rate of the ADS1292R chips needs to be set based on the Shimmers sampling rate
        if (samplingFrequency<=125):
            exgSamplingRate = 0x00 # 125Hz
        elif (samplingFrequency<=250):
            exgSamplingRate = 0x01 # 250Hz
        elif (samplingFrequency<=500):
            exgSamplingRate = 0x02 # 500Hz
        elif (samplingFrequency<=1000):
            exgSamplingRate = 0x03 # 1000Hz
        elif (samplingFrequency<=2000):
            exgSamplingRate = 0x04 # 2000Hz
        elif (samplingFrequency<=4000):
            exgSamplingRate = 0x05 # 4000Hz
        else:
            exgSamplingRate = 0x02 # 500Hz

        # Chip 1 configuration
        exgGainValueCh1 = 0x69
        exgGainValueCh2 = 0x60
        chip1Config = [exgSamplingRate, 0xA0, 0x10, exgGainValueCh1, exgGainValueCh2, 0x00, 0x00, 0x00, 0x02, 0x03]

        self.serial.flushInput()
        print("")
        print("####################################")
        print("###           Port open          ###")
        print("####################################")
        print("")

        # Get the daughter card ID byte (SR number)
        print("Requesting Daughter Card ID and Revision number...")
        self.serial.write(struct.pack('BBB', 0x66, 0x02,0x00))
        self.wait_for_ack()

        ddata = list(struct.unpack(4*'B', self.serial.read(4)))
        srNumber = ddata[2]
        srRev = ddata[3]

        print("Device: SR%d-%d" % (srNumber, srRev))

        # Send the set sampling rate command
        self.setSamplingRateHz(samplingFrequency)

        # Send the set sensors command (IMU and EMG)
        print("")
        print("Sending sensor commands for IMU and EMG communication...")
        self.serial.write(struct.pack('BBBBB', 0x08, 0xE0, 0x10, 0x00, 0x00)) # 0x08 is command byte, 0xE0 enables IMU, 0x10 enables EMG unit, 0x00 is trailing.
        self.wait_for_ack()
        time.sleep(2)

        if(srNumber == 47 and srRev >= 4):
            chip1Config[1] |= 8 # Config byte for CHIP1 in SR47-4


        # Configure Chip 1
        print("Sending chip configuration...")
        chip1Config = [0x61, 0x00, 0x00, 0x0A] + chip1Config
        self.serial.write(chip1Config)
        print("Chip configuration: ")
        print(chip1Config)
        print("")
        self.wait_for_ack()

        # send start streaming command
        print("Start streaming command sent...\n")
        print("")
        self.serial.write(struct.pack('B', 0x07))
        self.wait_for_ack()
        print("Now streaming data.")

    def wait_for_ack(self):
        ddata = b""
        ack = struct.pack('B', 0xff)
        while ddata != ack:
            ddata = self.serial.read(1)
        print("")
        return

    def setSamplingRateHz(self, rate=512):
        # send the set sampling rate command
        print("Sending sampling rate command...\n")
        sampling_freq = rate #Hz
        clock_wait = (2 << 14) // sampling_freq # Double // for integer division (python3)
        self.serial.write(struct.pack('<BH', 0x05, clock_wait))
        print("Sampling rate: ")
        print(sampling_freq)
        self.wait_for_ack()   

class Application(QtWidgets.QApplication):
    def __init__(self, sys_argv, node):
        super(Application, self).__init__(sys_argv)
        self.node = node

    def event(self, e):
        if rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0)
        return QtWidgets.QApplication.event(self, e)

def main(args=None):
    rclpy.init(args=args)
    ros_node = Node('emg_imu_node')
    app = Application(sys.argv, ros_node)
    signal.signal(signal.SIGINT, lambda *a: app.quit())
    ui = Main(ros_node)
    sys.exit(app.exec_())
