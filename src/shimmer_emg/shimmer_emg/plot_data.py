import pandas as pd
import matplotlib.pyplot as plt

# Load data from CSV
#data = pd.read_csv('../../../data_file.csv')
data = pd.read_csv('data_file.csv')

# Convert timestamp from seconds to minutes for readability
#data['timestamp'] = (data['timestamp'] - data['timestamp'].iloc[0]) / 60

# Determine unique states for coloring
unique_states = data['state'].unique()
state_colors = {'rest': 'white', 'flexion': 'green', 'static hold': 'grey', 'extension': 'blue', 'other': 'red'}


# Plotting
fig, axs = plt.subplots(4, 1, figsize=(10, 8), sharex=True)
fig.suptitle('EMG and IMU Data Over Time')

def plot_shaded_states(ax):
    prev_state = None
    start_time = None
    for i, (time, state) in enumerate(zip(data['timestamp'], data['state'])):
        if state != prev_state:
            if prev_state is not None:
                ax.axvspan(start_time, time, color=state_colors[prev_state], alpha=0.3)
            start_time = time
            prev_state = state
    # Cover the last segment
    if prev_state is not None:
        ax.axvspan(start_time, data['timestamp'].iloc[-1], color=state_colors[prev_state], alpha=0.3)



# EMG raw Data
axs[0].plot(data['timestamp'].values, data['emg_raw_ch1'].values, label='EMG Raw Ch1')
axs[0].plot(data['timestamp'].values, data['emg_raw_ch2'].values, label='EMG Raw Ch2')
axs[0].set_ylabel('Raw EMG')
axs[0].legend()
plot_shaded_states(axs[0])

# EMG filtered Data
axs[1].plot(data['timestamp'].values, data['emg_filtered_ch1'].values, label='EMG Filtered Ch1')
axs[1].plot(data['timestamp'].values, data['emg_filtered_ch2'].values, label='EMG Filtered Ch2')
axs[1].set_ylabel('Filtered EMG')
axs[1].legend()
plot_shaded_states(axs[1])

# Low Noise Accelerometer Data
axs[2].plot(data['timestamp'].values, data['ln_acc_x'].values, label='Acc X')
axs[2].plot(data['timestamp'].values, data['ln_acc_y'].values, label='Acc Y')
axs[2].plot(data['timestamp'].values, data['ln_acc_z'].values, label='Acc Z')
axs[2].set_ylabel('Low Noise Accel')
axs[2].legend()
plot_shaded_states(axs[2])

# Some features
axs[3].plot(data['timestamp'].values, data['emg_raw_mav_ch1'].values, label='MAV Raw Ch1')
axs[3].plot(data['timestamp'].values, data['emg_filtered_mav_ch1'].values, label='MAV Filtered Ch1')
axs[3].plot(data['timestamp'].values, data['acc_mav_x'].values, label='MAV Acc X')
axs[3].set_ylabel('MAV')
axs[3].legend()
plot_shaded_states(axs[2])

# Set common labels
for ax in axs:
    ax.set_xlabel('Time (seconds)')

plt.tight_layout(rect=[0, 0.03, 1, 0.95])
plt.show()