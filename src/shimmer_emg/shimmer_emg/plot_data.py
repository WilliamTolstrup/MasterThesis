import pandas as pd
import matplotlib.pyplot as plt

# Load data from CSV
data = pd.read_csv('continuous_file.csv')

# Determine unique states for coloring
unique_states = data['state'].unique()
state_colors = {'rest_heavy_vertical': 'white', 'flexion_heavy_vertical': 'green', 'extension_heavy_vertical': 'blue'}


# Plotting
fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
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

# Acc Data
axs[2].plot(data['timestamp'].values, data['ln_acc_x'].values, label='Accel x')
axs[2].plot(data['timestamp'].values, data['ln_acc_y'].values, label='Accel y')
axs[2].plot(data['timestamp'].values, data['ln_acc_z'].values, label='Accel z')
axs[2].set_ylabel('Accel')
axs[2].legend()
plot_shaded_states(axs[2])

# Set common labels
for ax in axs:
    ax.set_xlabel('Time (seconds)')

plt.tight_layout(rect=[0, 0.03, 1, 0.95])
plt.show()