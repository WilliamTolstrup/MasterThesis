# Master Thesis
Repository for master thesis on the control system for a soft exoskeleton based on the elbow joint.
The repository relies on ROS2 Humble.

It will contain scripts for acquiring EMG and IMU data, recording it to .csv files, plotting the data, and training a ML algorithm, and running it all on a Raspberry Pi 4

## ROADMAP

- [x] Acquire EMG and IMU data simultaneously
- [x] Calibrate IMU and convert data to usable units
- [x] Gather good elbow dataset for EMG and IMU data
- [x] Train a machine learning algorithm  (SVM) on the data
- [ ] Integrate encoder or potentiometer to estimate angle of the arm
- [ ] Integrate DC motor to actuate arm based on the model
- [ ] Make it work on a microcontroller
- [ ] Implmenet everything together to work with the exoskeleton


### Bluetooth setup
The bluetooth setup is a bit different depending on if you're connecting from a laptop or through SSH to a raspberry pi.

#### Laptop
To connect the shimmer device to the laptop, make sure you have the blueman bluetooth manager installed.
From the GUI, search for devices and when your shimmer device appears, trust and connect to it via the serial port (right click->serial port).
It will connect to '/dev/rfcomm0' by default, but if not, make sure to update the relevant scripts.
From the /info menu in the Blueman Bluetooth Manager, you will also find the MAC address for the device. Keep this handy, as it will be used for connecting via the Raspberry Pi.

#### Raspberry Pi
Since the application will be running on a headless installation of Linux, the bluetooth connection will need to be enabled through the terminal.
While most bluetooth specific packages should be standard with the Linux distribution, make sure you have the following:
dbus
bluez
bluez-tools
You can install these like so: 'sudo apt-get install bluez dbus bluez-tools'

You may also need to be the /dev/rfcomm0 serial port to the specific MAC address of your device, like so:
'sudo rfcomm bind 0 <MAC address> 1'

From here, SSH into the Raspberry Pi and run the following commands:
'bluetoothctl'
In here, you should run the following commands in order:
'power on'
'discoverable on'
'scan on'

When you see your device on the list (You may have to identify it with the MAC address), you run:
'connect <MAC address>'

It may disconnect within a few seconds, but that's alright, just leave the scan on.

### Acquisition of EMG and IMU data
If running on the laptop, remember to disable the ModemManager, as it interferes with the serial port.
    'sudo systemctl stop ModemManager'

To run the script, do the following:
Build and source the workspace
    'ros2 run shimmer_emg data_acquisition'

The following topics are published, and can be viewed in another terminal using the ros2 echo commands

    'ros2 topic echo *'

'*/shimmer/timestamp'
'*/emg/emg_raw'
'*/emg/emg_filtered'
'*/imu/ln_acc'
'*/shimmer/features'

Leave this script running

### Record data to .csv file
'ros2 run shimmer_emg record_data'

This script prints the following protocol to the terminal. It is important to follow the instructions as a correlating state is recorded and used for training the ML algorithm.

The following information is recorded and saved to a .csv file named 'data_file.csv':
timestamp
raw emg data (channel 1, channel 2)
filtered emg data (channel 1, channel 2)
low noise accelerometer (x,y,z)
Mean Absolute Value (MAV) for all channels
Root Mean Square Error (RMS) for all channels
Standard Deviation (SD) for all channels
Wavelength (WL) for all channels
State (rest, flexion, extension)

### Plot data from .csv file
cd into directory and run:
    'python3 plot_data'

Plots the data from the 'data_file.csv'. Must be in the directory when running.

### Train SVM
cd into directory and run:
    'python3 train_svm'

Only the features from the .csv file are used for training.
Currently, it trains both an LDA and SVM model, but just the SVM model is saved.
It uses the sigmoid kernel for the SVM model.

### Raspberry specific scripts
'ros2 run shimmer_emg rp_enable_dc'

This script loads the trained SVM model, it subscribes to the /shimmer/features topic and predicts the current state.
It controls the DC motor using PWM based on the state.
*** Currently, the encoder has not been implemented ***

### Running on Raspberry Pi
When SSH'ing into the raspberry pi, you will need 3 terminal:

1. Have the bluetooth setup running (It will probably not show that it is connected before running the next script)
2. Run the data acquisition script (Device should connect here)
3. Run the 'rp_enable_dc' script

This should enable the motor to turn either forward or backward depending on the predicted state. If "rest" or no state is predicted, the motor should remain stationary