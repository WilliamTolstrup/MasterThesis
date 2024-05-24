import pandas as pd
import matplotlib.pyplot as plt

# Load data from CSV
data = pd.read_csv('data_file.csv')

# Determine unique states for coloring
unique_states = data['state'].unique()
state_colors = {'rest_heavy_vertical': 'white', 'flexion_heavy_vertical': 'green', 'extension_heavy_vertical': 'blue'}


# Plotting
fig, axs = plt.subplots(8, 1, figsize=(10, 8), sharex=True)
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

# EMG rectified Data
axs[2].plot(data['timestamp'].values, data['emg_rectified_ch1'].values, label='EMG Rectified Ch1')
axs[2].plot(data['timestamp'].values, data['emg_rectified_ch2'].values, label='EMG Rectified Ch2')
axs[2].set_ylabel('Rectified EMG')
axs[2].legend()
plot_shaded_states(axs[2])

# EMG envelope Data
axs[3].plot(data['timestamp'].values, data['emg_envelope_ch1'].values, label='EMG Envelope Ch1')
axs[3].plot(data['timestamp'].values, data['emg_envelope_ch2'].values, label='EMG Envelope Ch2')
axs[3].plot(data['timestamp'].values, data['emg_envelope_combined'].values, label='EMG Envelope Combined')
axs[3].axhline(y=2.1082447e-8, color='r', linestyle='--', label='Threshold')
axs[3].set_ylabel('EMG Envelope')
axs[3].legend()
plot_shaded_states(axs[3])

# Acc Data
axs[4].plot(data['timestamp'].values, data['ln_acc_x'].values, label='Accel x')
axs[4].plot(data['timestamp'].values, data['ln_acc_y'].values, label='Accel y')
axs[4].plot(data['timestamp'].values, data['ln_acc_z'].values, label='Accel z')
axs[4].set_ylabel('Accel')
axs[4].legend()
plot_shaded_states(axs[4])

# EMG contraction Data
axs[5].plot(data['timestamp'].values, data['ch1_contraction'].values, label='Ch1 contractions')
axs[5].plot(data['timestamp'].values, data['ch2_contraction'].values, label='Ch2 contractions')
axs[5].plot(data['timestamp'].values, data['combined_contraction'].values, label='Combined contractions')
axs[5].set_ylabel('EMG Contractions (Bool)')
axs[5].legend()
plot_shaded_states(axs[5])

# Accel y derivative
axs[6].plot(data['timestamp'].values, data['acc_y_derivative'].values, label='Accel Y derivative')
axs[6].set_ylabel('Accel Derivative')
axs[6].legend()
plot_shaded_states(axs[6])

# Elbow Angle
axs[7].plot(data['timestamp'].values, data['elbow_angle'].values, label='Angle')
axs[7].set_ylabel('Estimated Elbow Angle')
axs[7].legend()
plot_shaded_states(axs[7])

# Set common labels
for ax in axs:
    ax.set_xlabel('Time (seconds)')

plt.tight_layout(rect=[0, 0.03, 1, 0.95])
plt.show()