import pandas as pd
import matplotlib.pyplot as plt

# Load data from CSV
#data = pd.read_csv('../../../data_file.csv')
data = pd.read_csv('data_file.csv')

#data = data[['timestamp', 'emg_raw_ch1', 'emg_raw_ch2', 'emg_filtered_ch1', 'emg_filtered_ch2', 'state']]#'ln_acc_x', 'ln_acc_y', 'ln_acc_z', 'state']]
# Convert timestamp from seconds to minutes for readability
#data['timestamp'] = (data['timestamp'] - data['timestamp'].iloc[0]) / 60

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

# # EMG raw MAV Data
# axs[2].plot(data['timestamp'].values, data['emg_raw_mav_ch1'].values, label='EMG raw MAV Ch1')
# axs[2].plot(data['timestamp'].values, data['emg_raw_mav_ch2'].values, label='EMG raw MAV Ch2')
# axs[2].set_ylabel('Raw MAV EMG')
# axs[2].legend()
# plot_shaded_states(axs[2])

# # EMG raw RMS Data
# axs[3].plot(data['timestamp'].values, data['emg_raw_rms_ch1'].values, label='EMG raw RMS Ch1')
# axs[3].plot(data['timestamp'].values, data['emg_raw_rms_ch2'].values, label='EMG raw RMS Ch2')
# axs[3].set_ylabel('Raw RMS EMG')
# axs[3].legend()
# plot_shaded_states(axs[3])

# # EMG raw SD Data
# axs[4].plot(data['timestamp'].values, data['emg_raw_sd_ch1'].values, label='EMG raw SD Ch1')
# axs[4].plot(data['timestamp'].values, data['emg_raw_sd_ch2'].values, label='EMG raw SD Ch2')
# axs[4].set_ylabel('Raw MAV EMG')
# axs[4].legend()
# plot_shaded_states(axs[4])

# # EMG raw wavelength Data
# axs[5].plot(data['timestamp'].values, data['emg_raw_wl_ch1'].values, label='EMG raw wavelength Ch1')
# axs[5].plot(data['timestamp'].values, data['emg_raw_wl_ch2'].values, label='EMG raw wavelength Ch2')
# axs[5].set_ylabel('Raw wavelength EMG')
# axs[5].legend()
# plot_shaded_states(axs[5])

# # EMG raw Coeffecients Data
# axs[6].plot(data['timestamp'].values, data['raw_coeff_1'].values, label='EMG raw coeff 1')
# axs[6].plot(data['timestamp'].values, data['raw_coeff_2'].values, label='EMG raw coeff 2')
# axs[6].plot(data['timestamp'].values, data['raw_coeff_3'].values, label='EMG raw coeff 3')
# axs[6].plot(data['timestamp'].values, data['raw_coeff_4'].values, label='EMG raw coeff 4')
# axs[6].plot(data['timestamp'].values, data['raw_coeff_5'].values, label='EMG raw coeff 5')
# axs[6].plot(data['timestamp'].values, data['raw_coeff_6'].values, label='EMG raw coeff 6')
# axs[6].plot(data['timestamp'].values, data['raw_coeff_7'].values, label='EMG raw coeff 7')
# axs[6].plot(data['timestamp'].values, data['raw_coeff_8'].values, label='EMG raw coeff 8')
# axs[6].set_ylabel('Raw Coeffs EMG')
# axs[6].legend()
# plot_shaded_states(axs[6])

# # Low Noise Accelerometer Data
# axs[2].plot(data['timestamp'].values, data['ln_acc_x'].values, label='Acc X')
# axs[2].plot(data['timestamp'].values, data['ln_acc_y'].values, label='Acc Y')
# axs[2].plot(data['timestamp'].values, data['ln_acc_z'].values, label='Acc Z')
# axs[2].set_ylabel('Low Noise Accel')
# axs[2].legend()
# plot_shaded_states(axs[2])

# Set common labels
for ax in axs:
    ax.set_xlabel('Time (seconds)')

plt.tight_layout(rect=[0, 0.03, 1, 0.95])
plt.show()