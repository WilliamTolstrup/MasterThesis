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
from std_msgs.msg import Bool, Empty, UInt8
from custom_interfaces.msg import EMG  # 

class Main(QtWidgets.QMainWindow):
    def __init__(self, node):
        super(Main, self).__init__()

        self.node = node  # ROS 2 node
        self.initUI() # GUI

        # Define serial object
        self.serialObj = serial.Serial()
        self.serialObj.baudrate = 115200
        self.serialObj.timeout = 1

        # Filter setup
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
        # Data buffers and thresholds
        self.init_buffers()

        # Serial thread and ROS 2 publishers/subscribers
        self.serial_handler = Communicate(self.serialObj)
        self.pubEmg = self.node.create_publisher(EMG, '/emg/emg', 10)
        self.pubRawEmg = self.node.create_publisher(EMG, '/emg/raw_emg', 10)
        self.pubEmgActivation = self.node.create_publisher(EMG, '/emg/emgActivation', 10)

        #delete
        self.pubPacket = self.node.create_publisher(UInt8, '/emg/packet', 10)

        self.node.create_subscription(Empty, '/emg/stop', self.stop_Btn, 10)
        self.node.create_subscription(UInt8, '/emg/start', self.start_Btn, 10)
        self.node.create_subscription(EMG, '/emg/activation_threshold', self.activation_threshold, 10)

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

    def init_buffers(self):
        self.i = 0
        self.windowSize = 50
        self.windowShift = 50
        self.movAveWindow = 40
        self.bufferShift_ch1 = []
        self.sBufferLast_ch1 = np.zeros(self.windowSize)
        self.sBufferfiltered_ch1 = np.zeros(self.windowSize)
        self.sBufferfiltered_lp_ch1 = np.zeros(self.windowSize)
        self.bufferShift_ch2 = []
        self.sBufferLast_ch2 = np.zeros(self.windowSize)
        self.sBufferfiltered_ch2 = np.zeros(self.windowSize)
        self.sBufferfiltered_lp_ch2 = np.zeros(self.windowSize)
        self.thresholdCh1 = 0.02
        self.thresholdCh2 = 0.02

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
            print('#########################################################')
            print('###         Reading serial port started               ###')
            print('#########################################################')
            self.allDataRaw = []
            self.allDataFiltered = []
        except:
            print('This Serial Port Is Not Available')

    def stop_Btn(self,data):
        print("terminate serial")
        if self.serialObj.is_open:
            self.serial_handler.reading = False
            self.thread1.exit()
            #send stop streaming command
            self.serialObj.write(struct.pack('B', 0x20))
            time.sleep(1)
            self.serial_handler.wait_for_ack() 
            time.sleep(1)
            self.serialObj.close() 
            
            
    def activation_threshold(self, data):
        self.thresholdCh1 = data.ch1
        self.thresholdCh2 = data.ch2

    
    def addData_callbackFunc(self, value1,value2, packettype):
        if self.i < self.windowShift:
            self.bufferShift_ch1.append(value1)
            self.bufferShift_ch2.append(value2)
            emg_t = EMG()

            emg_t.ch1 = self.sBufferfiltered_lp_ch1[self.i]
            emg_t.ch2 = self.sBufferfiltered_lp_ch2[self.i]  
            rawEmg = EMG()
            rawEmg.ch1 = value1
            rawEmg.ch2 = value2   
            self.pubEmg.publish(emg_t)
            self.pubRawEmg.publish(rawEmg)
            self.i += 1

        elif self.i == self.windowShift:
            self.i = 0
            # ##### for channel 1
            self.sBufferLast_ch1 = np.concatenate((self.sBufferLast_ch1[self.windowShift:],self.bufferShift_ch1))
            self.sBufferfiltered_ch1, self.zf_bp_ch1 = lfilter(self.b_bp, self.a_bp, self.sBufferLast_ch1, zi=self.zf_bp_ch1)
            self.sBufferfiltered_ch1, self.zf_bs_ch1 = lfilter(self.b_bs, self.a_bs, self.sBufferfiltered_ch1, zi=self.zf_bs_ch1)
            self.sBufferfiltered_lp_ch1, self.zf_lp_ch1 = lfilter(self.b_lp, self.a_lp, abs(self.sBufferfiltered_ch1), zi=self.zf_lp_ch1)
            activation_ch1 = np.mean(self.sBufferfiltered_lp_ch1)
            if activation_ch1 > self.thresholdCh1:
                print('muscle 1 is active: %2.3f' %activation_ch1)            
            self.bufferShift_ch1 = []

            # ##### for channel 2
            self.sBufferLast_ch2 = np.concatenate((self.sBufferLast_ch2[self.windowShift:],self.bufferShift_ch2))
            self.sBufferfiltered_ch2, self.zf_bp_ch2 = lfilter(self.b_bp, self.a_bp, self.sBufferLast_ch2, zi=self.zf_bp_ch2)
            self.sBufferfiltered_ch2, self.zf_bs_ch2 = lfilter(self.b_bs, self.a_bs, self.sBufferfiltered_ch2, zi=self.zf_bs_ch2)
            self.sBufferfiltered_lp_ch2, self.zf_lp_ch2 = lfilter(self.b_lp, self.a_lp, abs(self.sBufferfiltered_ch2), zi=self.zf_lp_ch2)
            activation_ch2 = np.mean(self.sBufferfiltered_lp_ch2)
            if activation_ch2 > self.thresholdCh2:
                print('muscle 2 is active: %2.3f' %activation_ch2)
            self.bufferShift_ch2 = []    

            # publishing the muscle activation
            emg_act = EMG()
            emg_act.ch1 = activation_ch1
            emg_act.ch2 = activation_ch2
            self.pubEmgActivation.publish(emg_act)

            packetMsg = UInt8()
            packetMsg.data = packettype
            self.pubPacket.publish(packetMsg)


class Communicate(QtCore.QObject):
    # A class for configuring the EMG module and reading the data from it
    data_signal = pyqtSignal(float, float, int)

    def __init__(self, serialObj):
        super(Communicate, self).__init__()
        self.reading = False
        self.serial = serialObj
        self.exgRes_24bit = True

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

        if self.exgRes_24bit:
            exgCalFactor = (((2.42 * 1000) / exgGain['GAIN_12']) / (pow(2, 23) - 1))
        else:
            exgCalFactor = (((2.42 * 1000) / (exgGain['GAIN_12'] * 2)) / (pow(2, 15) - 1))

        ddata = b""  # Use bytes literal for Python 3
        numbytes = 0
        framesize = 11  # 1byte packet type + 3byte timestamp + 14byte ExG data
        try:
            while self.reading:
                while numbytes < framesize:
                    ddata += self.serial.read(framesize)
                    numbytes = len(ddata)

                data = ddata[0:framesize]
                ddata = ddata[framesize:]
                numbytes = len(ddata)

                packettype = struct.unpack('B', data[0:1])[0]

                ts0, ts1, ts2, c1status = struct.unpack('BBBB', data[1:5])
                timestamp = ts0 + ts1 * 256 + ts2 * 65536

                if self.exgRes_24bit:
                    c1ch1 = struct.unpack('>i', data[5:8] + b'\0')[0] >> 8
                    c1ch2 = struct.unpack('>i', data[8:11] + b'\0')[0] >> 8
                else:
                    c1ch1, c1ch2 = struct.unpack('>hh', data[5:9])

                c1ch1 *= exgCalFactor
                c1ch2 *= exgCalFactor

                self.data_signal.emit(c1ch1, c1ch2, packettype)
        except KeyboardInterrupt:
            self.serial.write(struct.pack('B', 0x20))
            self.wait_for_ack()
            self.serial.close()
            print("All done!")

    def initialize_serial(self):
        # ExG configuration parameters
        exgconfigGain = {
                'GAIN_1':  0x15,
                'GAIN_2':  0x25,
                'GAIN_3':  0x35,
                'GAIN_4':  0x45,
                'GAIN_6':  0x50,
                'GAIN_8':  0x55,
                'GAIN_12': 0x65
        }

        exg_24bit = [0x10, 0x00, 0x00]
        exg_16bit = [0x00, 0x00, 0x18]

        '''
        ***************************
        * User settable variables *
        ***************************
        '''
        samplingFrequency 	= 1000						# frequency in Hz
        exgGainValue = exgconfigGain['GAIN_1'] 	# sets a gain of 1


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

        '''
        Example configuration for exg test signal square wave:

        Configure both ADS1292R (ExG) chips: 
         - enable internal voltage reference
         - 500 samples per second (internal sampling rate)
         - square-wave test signal

        write:
                SET_EXG_REGS_COMMAND
                chip identifier ('0' for chip1, '1' for chip2)
                starting byte
                number of bytes to write
                followed by the exgtestsignalconfiguration bytes


        where: 
                exgtestsignalconfiguration = ["CONFIG1" = 2, "CONFIG2" = 163, "LOFF" = 16,
                "CH1SET" = 5, "CH2SET" = 5, "RLD_SENS" = 0, "LOFF_SENS" = 0, "LOFF_STAT" = 0,
                "RESP1" = 2, "RESP2" = 1]

        '''
        # Chip 1 configuration
        exgGainValueCh1 = 0x69
        exgGainValueCh2 = 0x60
        chip1Config = [exgSamplingRate, 0xA0, 0x10, exgGainValueCh1, exgGainValueCh2, 0x00, 0x00, 0x00, 0x02, 0x03]

        # Chip 2 configuration
        #chip2Config = [exgSamplingRate, 0xA3, 0x10, exgGainValue, exgGainValue, 0x00, 0x00, 0x00, 0x02, 0x01]

        # connecting to serial port

        self.serial.flushInput()
        print("Port open...")

        #get the daughter card ID byte (SR number)
        print("Requesting Daughter Card ID and Revision number...")
        self.serial.write(struct.pack('BBB', 0x66, 0x02,0x00))
        self.wait_for_ack()

        ddata = list(struct.unpack(4*'B', self.serial.read(4)))
        srNumber = ddata[2]
        srRev = ddata[3]

        print("Device: SR%d-%d" % (srNumber, srRev))

        #send the set sensors command
        #self.serial.write(struct.pack('BBBB', 0x08, 0x10, 0x00, 0x00))  #exg1 and exg2
        #self.wait_for_ack()
        print("Sensor Enabling done...")

        # send the set sampling rate command
        self.setSamplingRateHz(samplingFrequency)
        print("Freq sent...")


        ''' 
            *************************
            * Send ExG config bytes *
            *************************
            '''
        # firstly - send the set sensors command - disable all
        if self.exgRes_24bit:
            sensors = [0x08] + exg_24bit
        else:
            sensors = [0x08] + exg_16bit
        self.serial.write(sensors)
        self.wait_for_ack()
        time.sleep(2)

        if(srNumber == 47 and srRev >= 4):
            chip1Config[1] |= 8 # Config byte for CHIP1 in SR47-4


        # Configure Chip 1
        chip1Config = [0x61, 0x00, 0x00, 0x0A] + chip1Config
        self.serial.write(chip1Config)
        print(chip1Config)
        self.wait_for_ack()

        # Configure Chip 2 
        #chip2Config = [0x61, 0x01, 0x00, 0x0A] + chip2Config
        #self.serial.write(chip2Config)
        #self.wait_for_ack()
        print("Configuration sent...")

        # send start streaming command
        self.serial.write(struct.pack('B', 0x07))
        self.wait_for_ack()
        print("Start sent...")

    def wait_for_ack(self):
        ddata = b""
        ack = struct.pack('B', 0xff)
        while ddata != ack:
            ddata = self.serial.read(1)

        return

    def setSamplingRateHz(self, rate=512):
        # send the set sampling rate command
        sampling_freq = rate #Hz
        clock_wait = (2 << 14) // sampling_freq # Double // for integer division (python3)
        self.serial.write(struct.pack('<BH', 0x05, clock_wait))
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
    ros_node = Node('emg_node')
    app = Application(sys.argv, ros_node)
    signal.signal(signal.SIGINT, lambda *a: app.quit())
    ui = Main(ros_node)
    sys.exit(app.exec_())
