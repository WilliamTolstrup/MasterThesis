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
from custom_interfaces.msg import EMG
from geometry_msgs.msg import Vector3

class Main(QtWidgets.QMainWindow):
    def __init__(self, node):
        super(Main, self).__init__()

        self.node = node  # ROS 2 node
        self.initUI() # GUI

        # Define serial object
        self.serialObj = serial.Serial()
        self.serialObj.baudrate = 115200
        self.serialObj.timeout = 1

        # Serial thread and ROS 2 publishers/subscribers
        self.serial_handler = Communicate(self.serialObj)
        self.pubAcc = self.node.create_publisher(Vector3, '/shimmer/acc', 10)
        self.pubGyro = self.node.create_publisher(Vector3, '/shimmer/gyro', 10)
        self.pubMag = self.node.create_publisher(Vector3, '/shimmer/mag', 10)

        self.node.create_subscription(Empty, '/shimmer/stop', self.stop_Btn, 10)
        self.node.create_subscription(UInt8, '/shimmer/start', self.start_Btn, 10)

        self.thread1 = QtCore.QThread()
        self.serial_handler.moveToThread(self.thread1)
        self.thread1.started.connect(self.serial_handler.dataSendLoop)
        self.serial_handler.data_signal.connect(self.publish_data)

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
            
    def publish_data(self, accX, accY, accZ, gyroX, gyroY, gyroZ, magX, magY, magZ):
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
        
        self.pubAcc.publish(accMsg)
        self.pubGyro.publish(gyroMsg)
        self.pubMag.publish(magMsg)

class Communicate(QtCore.QObject):
    # A class for configuring the EMG module and reading the data from it
    data_signal = pyqtSignal(float,float,float,float,float,float,float,float,float, int)

    def __init__(self, serialObj):
        super(Communicate, self).__init__()
        self.reading = False
        self.serial = serialObj
        self.exgRes_24bit = True

    def dataSendLoop(self):
        self.initialize_serial()

        ddata = b""  # Use bytes literal for Python 3
        numbytes = 0
        framesize = 22  # 1byte packet type + 3byte timestamp + 14byte ExG data
        try:
            while self.reading:
                while numbytes < framesize:
                    ddata += self.serial.read(framesize)
                    numbytes = len(ddata)

                data = ddata[0:framesize]
                ddata = ddata[framesize:]
                numbytes = len(ddata)

                packet = struct.unpack('B', data[0:1])[0]

                #ts0, ts1, ts2, c1status = struct.unpack('BBBB', data[1:3]) #was 1:5
                #timestamp = ts0 + ts1 * 256 + ts2 * 65536

                #(packettype,) = struct.unpack('B', data[0:1])
                (timestamp,) = struct.unpack('H', data[1:3])
                (accx, accy, accz) = struct.unpack('HHH', data[3:9])
                (gyrox, gyroy, gyroz) = struct.unpack('HHH', data[9:15])
                (magx, magy, magz) = struct.unpack('>hhh', data[15:21])
                #print "0x%02x,%5d,\t%4d, %4d, %4d, \t%4d, %4d, %4d, \t%4d, %4d, %4d" % (packettype, timestamp,accx, accy, accz, gyrox, gyroy, gyroz, magx, magy, magz)
                self.data_signal.emit(accx, accy, accz, gyrox, gyroy, gyroz, magx, magy, magz, packet) # <- Here you emit a signal!
        
        except KeyboardInterrupt:
            self.serial.write(struct.pack('B', 0x20))
            self.wait_for_ack()
            self.serial.close()
            print("All done!")

    def initialize_serial(self):
        print("")
        print("##########################################")
        print("###           PORT OPEN                ###")
        print("##########################################")
        print("")
        # firstly - send the set sensors command - disable all
        self.serial.flushInput()
        # send the set sensors command
        self.serial.write(struct.pack('BBBB', 0x08, 0xE0, 0x00, 0x00))  #analog accel, gsr, MPU9150 gyro
        self.wait_for_ack()
        # send the set sampling rate command
        self.serial.write(struct.pack('BBB', 0x05, 0x80, 0x02)) #51.2Hz (32768/640=51.2Hz: 640 -> 0x0280; has to be done like this for alignment reasons.)
        self.wait_for_ack()
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
    ros_node = Node('imu_node')
    app = Application(sys.argv, ros_node)
    signal.signal(signal.SIGINT, lambda *a: app.quit())
    ui = Main(ros_node)
    sys.exit(app.exec_())
