import pandas as pd
import matplotlib.pyplot as plt

# Load data from CSV
data = pd.read_csv('../../../data_file.csv')

# Convert timestamp from seconds to minutes for readability
data['timestamp'] = (data['timestamp'] - data['timestamp'].iloc[0]) / 60

# Plotting
fig, axs = plt.subplots(4, 1, figsize=(10, 8), sharex=True)
fig.suptitle('EMG and IMU Data Over Time')

# EMG Data
axs[0].plot(data['timestamp'].values, data['emg_ch1'].values, label='EMG Ch1')
axs[0].plot(data['timestamp'].values, data['emg_ch2'].values, label='EMG Ch2')
axs[0].set_ylabel('EMG')
axs[0].legend()

# Accelerometer Data
axs[1].plot(data['timestamp'].values, data['acc_x'].values, label='Acc X')
axs[1].plot(data['timestamp'].values, data['acc_y'].values, label='Acc Y')
axs[1].plot(data['timestamp'].values, data['acc_z'].values, label='Acc Z')
axs[1].set_ylabel('Accel')
axs[1].legend()

# Gyroscope Data
axs[2].plot(data['timestamp'].values, data['gyro_x'].values, label='Gyro X')
axs[2].plot(data['timestamp'].values, data['gyro_y'].values, label='Gyro Y')
axs[2].plot(data['timestamp'].values, data['gyro_z'].values, label='Gyro Z')
axs[2].set_ylabel('Gyro')
axs[2].legend()

# Magnetometer Data
axs[3].plot(data['timestamp'].values, data['mag_x'].values, label='Mag X')
axs[3].plot(data['timestamp'].values, data['mag_y'].values, label='Mag Y')
axs[3].plot(data['timestamp'].values, data['mag_z'].values, label='Mag Z')
axs[3].set_ylabel('Mag')
axs[3].legend()

# Set common labels
for ax in axs:
    ax.set_xlabel('Time (minutes)')

plt.tight_layout(rect=[0, 0.03, 1, 0.95])
plt.show()
