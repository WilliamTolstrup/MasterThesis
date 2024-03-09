# Master Thesis
Repo for my master thesis on a soft exoskeleton for the elbow joint. This repo will mostly consist of the control system design

Currently, the custom_interfaces pkg is not used, as there was a problem with integrating it with the python code. Instead, a generic Vector3 .msg has been used for the EMG signals, using .x for ch1 and .y for ch2.

## shimmer_emg/shimmer_emg/imu_emg_aq.py
Combined script that connects with the shimmer3 unit and outputs both EMG and IMU data to ROS2 topics.

To run:
Build and source the workspace.
Connect shimmer device via bluetooth. Ensure that the connected serial port is '/dev/rfcomm0'. If not, change it in the code the start_Btn() function.
When it is connected, run: ros2 run shimmer_emg imu_emg_aq
A GUI will pop up, and pressing "Start" a connection between the shimmer device is attempted.

In another terminal, you can run these commands to see the outpot:
ros2 topic echo *
*/emg/emg_raw
*/emg/emg_filtered
*/imu/acc
*/imu/gyro
*/imu/mag

Before running the script, disable ModemManager, as it may interfere with the serial port.
    sudo systemctl stop ModemManager



#### ROADMAP

* Acquire EMG and IMU data [Checkmark]
Acquire both EMG and IMU data simultaneously

* Machine learning
Using simpler models suchs as SVMs and smaller libraries like scikit-learn, tensorflow lite, or pytorch mobile, to create a model that can accurately predict flexion, extension, or static hold.

* Integrate encoder to estimate angle of arm

* Integrate DC motor to actuate arm based on the model