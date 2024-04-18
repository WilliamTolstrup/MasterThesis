# Master Thesis
Repository for master thesis on the control system for a soft exoskeleton based on the elbow joint.
The repository relies on ROS2 Humble.

It will contain scripts for acquiring EMG and IMU data, recording it to .csv files, plotting the data, and training a ML algorithm

Currently, the custom_interfaces pkg is not used, as there was a problem with integrating it with the python code. Instead, a generic Vector3 .msg has been used for the EMG signals, using .x for ch1 and .y for ch2.

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

### Acquisition of EMG and IMU data
'ros2 run shimmer_emg data_acquisition'
Combined script that connects with the shimmer3 unit and outputs both EMG and IMU data to ROS2 topics.

To run:
Build and source the workspace.
Connect shimmer device via bluetooth. Ensure that the connected serial port is '/dev/rfcomm0'. If not, change it in the code the start_Btn() function.
A GUI will pop up, and pressing "Start" a connection between the shimmer device is attempted.
When the data stream is connected, you can calibrate the device by pressing the "Calibrate" button, and follow the instructions on the screen.

In another terminal, you can run these commands to see the outpot:
'ros2 topic echo *'
*/emg/emg_raw
*/emg/emg_filtered
*/imu/acc
*/imu/gyro
*/imu/mag

Before running the script, disable ModemManager, as it may interfere with the serial port.
    'sudo systemctl stop ModemManager'

### Record data to .csv file
'ros2 run shimmer_emg record_data'

Records the timestamp, emg channel 1 and 2, accelerometer x,y,z, gyroscope x,y,z, magnetometer x,y,z to a .csv file named 'data_file.csv'

### Plot data from .csv file
'python3 plot_data'

Plots the data from the 'data_file.csv'. Must be in the directory when running.

