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
import subprocess
import json
from filterpy.kalman import MerweScaledSigmaPoints

class Main(QtWidgets.QMainWindow):
    def __init__(self, node):
        super(Main, self).__init__()

        self.node = node  # ROS 2 node
        
        self.errorBias = [0, 0, 0, 0, 0, 0, 0, 0, 0] # Accel, gyro, mag, xyz

        self.coefficients = {
            'acc_x': None,
            'acc_y': None,
            'acc_z': None,
            'gyro_x': None,
            'gyro_y': None,
            'gyro_z': None
        }

        # Define serial object
        self.serialObj = serial.Serial()
        self.serialObj.baudrate = 115200
        self.serialObj.timeout = 1

        # ROS2 Publisher
        # Timestamp
        self.pubTimestamp   = self.node.create_publisher(Time, '/shimmer/timestamp', 10)
        # EMG
        self.pubEmgRaw      = self.node.create_publisher(Vector3, '/emg/emg_raw', 10)
        self.pubEmgFiltered = self.node.create_publisher(Vector3, '/emg/emg_filtered', 10)
        # IMU
        self.pubAcc  = self.node.create_publisher(Vector3, '/imu/acc', 10)
        self.pubGyro = self.node.create_publisher(Vector3, '/imu/gyro', 10)
        self.pubMag  = self.node.create_publisher(Vector3, '/imu/mag', 10)

        # ROS2 Subscription
        # EMG and IMU
        self.node.create_subscription(Empty, '/shimmer/stop', self.stop_Btn, 10)
        self.node.create_subscription(UInt8, '/shimmer/start', self.start_Btn, 10)

        # Serial thread and ROS 2 publishers/subscribers
        self.serial_handler = Communicate(self.serialObj, self.node)
        self.calibration_service = CalibrationService(node, self.serial_handler)

        # Data thread
        self.dataThread = QtCore.QThread()
        self.serial_handler.moveToThread(self.dataThread)
        self.dataThread.started.connect(self.serial_handler.dataSendLoop)
        self.serial_handler.data_signal.connect(self.addData_callbackFunc)

        # Calibration thread
        self.calibrationThread = QtCore.QThread()
        self.calibration_service.moveToThread(self.calibrationThread)

        self.setup_signals() # Message passing between classes
        self.setup_filters() # EMG filters
        self.initUI()        # GUI
        self.setup_sma()     # Simple moving average

    def load_calibration_data(filepath='calibration_data.json'):
        with open(filepath, 'r') as f:
            data = json.load(f)
        return data['coefficients']

    def setup_sma(self): # Simple Moving Average
        IMU_window = 50 #23040 # baudrate / 5
        EMG_window = 25
        self.accX_sma = RealTimeMovingAverage(window_size=IMU_window)
        self.accY_sma = RealTimeMovingAverage(window_size=IMU_window)
        self.accZ_sma = RealTimeMovingAverage(window_size=IMU_window)

        self.gyroX_sma = RealTimeMovingAverage(window_size=IMU_window)
        self.gyroY_sma = RealTimeMovingAverage(window_size=IMU_window)
        self.gyroZ_sma = RealTimeMovingAverage(window_size=IMU_window)

        self.magX_sma = RealTimeMovingAverage(window_size=IMU_window)
        self.magY_sma = RealTimeMovingAverage(window_size=IMU_window)
        self.magZ_sma = RealTimeMovingAverage(window_size=IMU_window)

        self.emg_ch1_sma = RealTimeMovingAverage(window_size=EMG_window)
        self.emg_ch2_sma = RealTimeMovingAverage(window_size=EMG_window)

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

    def setup_signals(self):
        self.calibration_service.calibrationMessage.connect(self.update_calibration_message)
        self.calibration_service.calibrationChanged.connect(self.update_calibration_status)
        self.calibration_service.calibrationComplete.connect(self.update_calibration_button)
        self.serial_handler.calibration_signal.connect(self.calibration_service.handle_calibration_buffer)
        self.calibration_service.errorBiasUpdate.connect(self.update_error_bias)
        self.calibration_service.coefficientsUpdate.connect(self.update_coefficients)

    def update_error_bias(self, bias):
        self.errorBias = bias

    def update_coefficients(self, coefficients):
        self.coefficients = coefficients

    def update_calibration_message(self, title, message):
        QtWidgets.QMessageBox.information(self, title, message)

    def update_calibration_status(self, message, color):
        self.calibrateStatus.setText(message)
        self.calibrateStatus.setStyleSheet(f"QLabel {{ color: {color}; }}")

    def update_calibration_button(self, status):
        self.calibrateBtn.setEnabled(status)

    def initUI(self):
        self.setWindowTitle("Shimmer Controller")
        self.setGeometry(100, 100, 800, 600)

        # Start button
        self.startBtn = QtWidgets.QPushButton("Start", self)
        self.startBtn.move(50, 500)
        self.startBtn.resize(100, 40)
        self.startBtn.clicked.connect(self.start_Btn)

        # Stop button
        self.stopBtn = QtWidgets.QPushButton("Stop", self)
        self.stopBtn.move(155, 500)
        self.stopBtn.resize(100, 40)
        self.stopBtn.clicked.connect(self.stop_Btn)

        # Calibrate button
        self.calibrateBtn = QtWidgets.QPushButton("Calibrate", self)
        self.calibrateBtn.move(260, 500)
        self.calibrateBtn.resize(200, 40)
        self.calibrateBtn.clicked.connect(self.calibrate_Btn)

        # Status label
        self.statusLabel = QtWidgets.QLabel("Disconnected", self)
        self.statusLabel.move(50, 50)
        self.statusLabel.resize(200, 40)
        self.statusLabel.setStyleSheet("QLabel {color : red; }")

        # Calibration status label
        self.calibrateStatus = QtWidgets.QLabel("Not calibrated", self)
        self.calibrateStatus.move(50, 100)
        self.calibrateStatus.resize(800, 80)
        self.calibrateStatus.setStyleSheet("QLabel {color : red; }")

        # Show the window
        self.show()

    ############################################################
    ###                      GUI Buttons                     ###
    ############################################################

    def start_Btn(self):
        try:
            # Static port number. Remember to change if using a different unit. 5F90.
            port = str('/dev/rfcomm0')
            print(port)
            self.serialObj.port = port
            self.serialObj.open()
            self.serial_handler.reading = True
            self.dataThread.start()
            print('####################################')
            print('###  Reading serial port started ###')
            print('####################################')
            self.statusLabel.setText("Connected")
            self.statusLabel.setStyleSheet("QLabel {color : green; }")

        except:
            print('Error STA1: This Serial Port Is Not Available')

    def stop_Btn(self,data):
        print("####################################")
        print("### Serial connection terminated ###")
        print("####################################")
        if self.serialObj.is_open:
            self.serial_handler.reading = False
            self.dataThread.exit()
            #send stop streaming command
            self.serialObj.write(struct.pack('B', 0x20))
            time.sleep(1)
            self.serial_handler.wait_for_ack() 
            time.sleep(1)
            self.serialObj.close() 
            self.statusLabel.setText("Disconnected")
            self.statusLabel.setStyleSheet("QLabel {color : red; }")

    def calibrate_Btn(self):
        if not self.serial_handler.reading:
            QtWidgets.QMessageBox.warning(self, "Warning!", "The device is not connected, please connect before calibrating!")
            print("Error C1: Please connect device before calibrating!")
            return

        else:
            self.calibrationThread.start()
            # Display static calibration instructions and wait
            print("Initiating calibration...")
            self.calibration_service.start_calibration()

    ############################################################
    ###                    Data callback                     ###
    ############################################################

    def addData_callbackFunc(self, timestamp_sec, timestamp_nsec, ch1, ch2, accX, accY, accZ, gyroX, gyroY, gyroZ, magX, magY, magZ):
        # Timestamp
        timestampMsg = Time()
        timestampMsg.sec = timestamp_sec
        timestampMsg.nanosec = timestamp_nsec
        
        # EMG
        # Create objects for raw and filtered EMG data
        emg_raw_msg = Vector3()
        emg_filtered_msg = Vector3()

        # Assign raw values
        emg_raw_msg.x = ch1
        emg_raw_msg.y = ch2

        # Step 1: Bandpass filter
        bp_ch1, self.zf_bp_ch1 = lfilter(self.b_bp, self.a_bp, [ch1], zi=self.zf_bp_ch1)
        bp_ch2, self.zf_bp_ch2 = lfilter(self.b_bp, self.a_bp, [ch2], zi=self.zf_bp_ch2)

        # Step 2: Bandstop filter
        bs_ch1, self.zf_bs_ch1 = lfilter(self.b_bs, self.a_bs, bp_ch1, zi=self.zf_bs_ch1)
        bs_ch2, self.zf_bs_ch2 = lfilter(self.b_bs, self.a_bs, bp_ch2, zi=self.zf_bs_ch2)

        # Step 3: Lowpass filter
        lp_ch1, self.zf_lp_ch1 = lfilter(self.b_lp, self.a_lp, bs_ch1, zi=self.zf_lp_ch1)
        lp_ch2, self.zf_lp_ch2 = lfilter(self.b_lp, self.a_lp, bs_ch2, zi=self.zf_lp_ch2)

        # Step 4: Simple moving average
        sma_ch1 = self.emg_ch1_sma.update(lp_ch1)
        sma_ch2 = self.emg_ch2_sma.update(lp_ch2)

        # Assign filtered values
        emg_filtered_msg.x = sma_ch1[0]#lp_ch1[0]  # lfilter returns a list, take the first element
        emg_filtered_msg.y = sma_ch2[0]#lp_ch2[0]  # Same here


        # IMU
        # Moving average filter on incoming data
        sma_accX = self.accX_sma.update(accX)
        sma_accY = self.accX_sma.update(accY)
        sma_accZ = self.accX_sma.update(accZ)

        sma_gyroX = self.gyroX_sma.update(gyroX)
        sma_gyroY = self.gyroX_sma.update(gyroY)
        sma_gyroZ = self.gyroX_sma.update(gyroZ)

        sma_magX = self.magX_sma.update(magX)
        sma_magY = self.magX_sma.update(magY)
        sma_magZ = self.magX_sma.update(magZ)


        a = 1#1.030177513657288994


        corrected_accX = sma_accX - (self.errorBias[0] * a)
        corrected_accY = sma_accY - (self.errorBias[1] * a)
        corrected_accZ = sma_accZ - (self.errorBias[2] * a)

        corrected_gyroX = sma_gyroX - (self.errorBias[3] * a)
        corrected_gyroY = sma_gyroY - (self.errorBias[4] * a)
        corrected_gyroZ = sma_gyroZ - (self.errorBias[5] * a)

        corrected_magX = sma_magX - self.errorBias[6]
        corrected_magY = sma_magY - self.errorBias[7]
        corrected_magZ = sma_magZ    # No mag_z errorbias, since we did 2D calibration

        # # Compute the time step since the last update
        # current_timestamp = timestamp_sec + timestamp_nsec * 1e-9  # Convert to seconds
        # if hasattr(self, 'previous_timestamp'):
        #     dt = current_timestamp - self.previous_timestamp
        # else:
        #     dt = 0  # No previous timestamp, so assume zero (or some small value)
        # self.previous_timestamp = current_timestamp

        # # Update the UKF with corrected gyroscope data along the Y-axis
        # estimated_angle = self.ukf.update(corrected_gyroY, dt)

        # corrected_accX = accX
        # corrected_accY = accY
        # corrected_accZ = accZ
        # corrected_gyroX = gyroX
        # corrected_gyroY = gyroY
        # corrected_gyroZ = gyroZ
        # corrected_magX = magX
        # corrected_magY = magY
        # corrected_magZ = magZ

        # IMU
        accMsg = Vector3()
        gyroMsg = Vector3()
        magMsg = Vector3()

        accMsg.x = corrected_accX
        accMsg.y = corrected_accY
        accMsg.z = corrected_accZ
        
        gyroMsg.x = corrected_gyroX
        gyroMsg.y = corrected_gyroY
        gyroMsg.z = corrected_gyroZ
        
        magMsg.x = corrected_magX
        magMsg.y = corrected_magY
        magMsg.z = corrected_magZ        
        
        # Publish
        self.pubTimestamp.publish(timestampMsg)
        self.pubAcc.publish(accMsg)
        self.pubGyro.publish(gyroMsg)
        self.pubMag.publish(magMsg)
        self.pubEmgRaw.publish(emg_raw_msg)
        self.pubEmgFiltered.publish(emg_filtered_msg)

class CalibrationService(QtCore.QObject):
    calibrationChanged = pyqtSignal(str, str) # Message and color
    calibrationComplete = pyqtSignal(bool) # True or False, whether to enable the calibrate button
    calibrationInProgress = pyqtSignal(bool) # True of False, whether the calibration is in progress or not
    calibrationProgress = pyqtSignal(int) # Remaining time in seconds
    calibrationMessage = pyqtSignal(str, str) # Title, message
    errorBiasUpdate = pyqtSignal(list)
    coefficientsUpdate = pyqtSignal(dict)

    def __init__(self, node, serial_handler):
        super().__init__()
        self.node = node
        self.serial_handler_calibration = serial_handler
        self.calibrationInProgress.emit(False)
        self.calibration_static_complete = False
        self.calibration_buffer = []
        self.errorBias = [0, 0, 0, 0, 0, 0]
        self.coefficients = {
            'acc_x': None,
            'acc_y': None,
            'acc_z': None,
            'gyro_x': None,
            'gyro_y': None,
            'gyro_z': None
        }

    def handle_calibration_buffer(self, buffer):
        self.calibration_buffer = buffer


    def start_calibration(self):
        self.calibration_buffer.clear()
        self.calibrationComplete.emit(False) # Disable calibration button while calibrating
        self.calibrationInProgress.emit(True)

        # Calculate average of accelerometer and gyro
        if self.calibration_static_complete == False:
            self.calibrationChanged.emit("Static calibration!\nLeave the device level and stationary for 10 seconds.", "blue")
            QtCore.QTimer.singleShot(10000, self.complete_static_calibration) # Wait 10 seconds and then complete calibration

        # HSI calibration for magnetometer
        elif self.calibration_static_complete == True:
            self.calibrationChanged.emit("Dynamic calibration!\nRotate the device slowly for 10 seconds.", "blue")
            QtCore.QTimer.singleShot(10000, self.complete_dynamic_calibration) # Wait 10 seconds and then complete calibration

        else:
            self.calibrationChanged.emit("An error may have occured!\nRestart the device and try again.", "red")

    def complete_static_calibration(self):
        print("Static calibration complete!")
        self.calibration_static_complete = True

        # Calculate average for calibration
        if self.calibration_buffer:
            acc_x = [x[0] for x in self.calibration_buffer]
            acc_y = [y[1] for y in self.calibration_buffer]
            acc_z = [z[2] for z in self.calibration_buffer]

            gyro_x = [x[3] for x in self.calibration_buffer]
            gyro_y = [y[4] for y in self.calibration_buffer]
            gyro_z = [z[5] for z in self.calibration_buffer]

            acc_bias_x = sum(acc_x) / len(acc_x)
            acc_bias_y = sum(acc_y) / len(acc_y)
            acc_bias_z = sum(acc_z) / len(acc_z)

            gyro_bias_x = sum(gyro_x) / len(gyro_x)
            gyro_bias_y = sum(gyro_y) / len(gyro_y)
            gyro_bias_z = sum(gyro_z) / len(gyro_z)

            self.errorBias[0:6] = [acc_bias_x, acc_bias_y, acc_bias_z, gyro_bias_x, gyro_bias_y, gyro_bias_z]
            self.calibrationChanged.emit("Static calibration complete", "orange")

            time.sleep(3)
            self.start_calibration()
        
    def complete_dynamic_calibration(self):
        print("Dynamic calibration complete!")
        # Perform HSI calibration for the mag data
        if self.calibration_buffer:
            # Hard iron calibration
            mag_x = [x[-3] for x in self.calibration_buffer]
            mag_y = [y[-2] for y in self.calibration_buffer]

            # Calculate average bias
            bias_x = sum(mag_x) / len(mag_x)
            bias_y = sum(mag_y) / len(mag_y)

            self.errorBias[7:8] = [bias_x, bias_y]

            # Notify the user
            self.calibrationChanged.emit("Dynamic calibration complete", "orange")
            self.errorBiasUpdate.emit(self.errorBias)

            self.calibrationComplete.emit(True) # Enable calibration button after calibrating
            self.calibrationInProgress.emit(False)

            print("")
            print("Error bias: \n")
            print(self.errorBias)

            self.calibrationChanged.emit("Calibration complete", "green")
            print("####################################")
            print("###     Calibration complete!    ###")
            print("####################################")

class Communicate(QtCore.QObject):
    # A class for configuring the EMG module and reading the data from it
    data_signal = pyqtSignal(int, int, float, float, float, float, float, float, float, float, float, float, float)
    calibration_signal = pyqtSignal(list)

    def __init__(self, serialObj, node):
        super(Communicate, self).__init__()
        self.reading = False
        self.serial = serialObj
        self.node = node
        self.calibration_buffer = []


    def dataSendLoop(self):
        self.initialize_serial()
        gain = 12 # EMG gain 1,2,3,4,6,8,12
#		      ADC Sensivitivty   (try removing)
        exgCalFactor = (((2.42 * 1000) / gain) / (pow(2, 23) - 1))
        2420 / (2 ** 23 - 1)

        ddata = b""  # Use bytes literal for Python 3
        numbytes = 0
        framesize = 28#29  # 1 byte packet type + 4 bytes timestamp + 6 bytes emg, 18 bytes imu (6x3 acc, gyro, mag)

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

                print("DATA: ")
                print(data)

                packettype = struct.unpack('B', data[0:1])
                (ts0, ts1,) = struct.unpack('BB', data[1:3])
                print("packet, ts0, ts1:")
                print(packettype, ts0, ts1)
                (accx, accy, accz) = struct.unpack('HHH', data[3:9])
                (gyrox, gyroy, gyroz) = struct.unpack('HHH', data[9:15])
                (magx, magy, magz) = struct.unpack('>hhh', data[15:21])
                ch1 = struct.unpack('>i', data[21:24] + b'\0')[0] >> 8 # 24 Bit size, so we add a null byte on the end 
                ch2 = struct.unpack('>i', data[24:27] + b'\0')[0] >> 8 # and then shift the data 8 bits right to remove it again

                ch1 *= exgCalFactor
                ch2 *= exgCalFactor

                # Create timestamp
                current_time = self.node.get_clock().now()
                timestamp_sec = current_time.seconds_nanoseconds()[0]
                timestamp_nsec = current_time.seconds_nanoseconds()[1]

                # Conversions
                # Convert raw data to voltage
                voltage_x = (accx / adc_resolution) * reference_voltage
                voltage_y = (accy / adc_resolution) * reference_voltage
                voltage_z = (accz / adc_resolution) * reference_voltage

                # Convert voltage to gs
                accX = (voltage_x - zero_g_voltage) / sensitivity_v_per_g
                accY = (voltage_y - zero_g_voltage) / sensitivity_v_per_g
                accZ = (voltage_z - zero_g_voltage) / sensitivity_v_per_g

                # Convert to degrees/sec
                gyroX = (gyrox/gyro_sensitivity)
                gyroY = (gyroy/gyro_sensitivity)
                gyroZ = (gyroz/gyro_sensitivity)

                # Convert to gs
                magX = (magx / mag_sensitivity_xy)
                magY = (magy / mag_sensitivity_xy)
                magZ = (magz / mag_sensitivity_z)

                #accx, accy, accz, gyrox, gyroy, gyroz, magx, magy, magz
                #accX, accY, accZ, gyroX, gyroY, gyroZ, magX, magY, magZ

                self.calibration_buffer.append([accX, accY, accZ, gyroX, gyroY, gyroZ, magX, magY, magZ])

                self.data_signal.emit(timestamp_sec, timestamp_nsec, ch1, ch2, accx, accy, accz, gyrox, gyroy, gyroz, magx, magy, magz)
                self.calibration_signal.emit(self.calibration_buffer)

                #print("ch2: ")
                #print(ch2)
                #print(magY)
                #print(magZ)
                # GyroZ has issues. goes below 0 and restarts at 500


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
        print("Sending sensor commands for IMU and EMG communication...") # 0x90 enables emg (24bit, chip 1) and accelerometer (low noise)
        self.serial.write(struct.pack('BBBB', 0x08, 0x90, 0x00, 0x00)) # 0x08 is command byte, 0x10 enables EMG, 0x00 is trailing.  0xF0 Enables low noise accel, gyro, mag, exg
        #self.serial.write(struct.pack('BBBB', 0x08, 0xE0, 0x00, 0x00)) # 0x08 is command byte, 0xE0 enables IMU, 0x00 is trailing.
        self.wait_for_ack()
        time.sleep(2)

        # send the set sampling rate command
        self.serial.write(struct.pack('BBB', 0x05, 0x80, 0x02)) #51.2Hz (32768/640=51.2Hz: 640 -> 0x0280; has to be done like this for alignment reasons.)
        self.wait_for_ack()

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


class RealTimeMovingAverage:
    def __init__(self, window_size):
        self.window_size = window_size
        self.data = deque(maxlen=window_size)
        self.sum = 0.0

    def update(self, new_value):
        if len(self.data) == self.window_size:
            self.sum -= self.data[0] # Remove oldest value from the sum
        
        self.data.append(new_value)
        self.sum += new_value # Add newest data to the sum

        return self.sum / len(self.data) # Return average of the window

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
