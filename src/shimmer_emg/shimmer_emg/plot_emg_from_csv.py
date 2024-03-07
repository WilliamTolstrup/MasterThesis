import matplotlib.pyplot as plt
import pandas as pd

def plot_emg_data(csv_file_path):
    # Load the CSV file into a DataFrame
    df = pd.read_csv(csv_file_path, header=None, names=['Channel 1', 'Channel 2'])

    # Plotting
    plt.figure(figsize=(10, 6))

    # Plot Channel 1
    plt.subplot(2, 1, 1)  # 2 rows, 1 column, 1st subplot
    plt.plot(df['Channel 1'], label='Channel 1')
    plt.title('EMG Data - Channel 1')
    plt.ylabel('Amplitude')
    plt.grid(True)
    plt.legend()

    # Plot Channel 2
    plt.subplot(2, 1, 2)  # 2 rows, 1 column, 2nd subplot
    plt.plot(df['Channel 2'], label='Channel 2')
    plt.title('EMG Data - Channel 2')
    plt.xlabel('Sample')
    plt.ylabel('Amplitude')
    plt.grid(True)
    plt.legend()

    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    csv_file_path = 'emg_data.csv'  # Update this if your CSV file is located elsewhere
    plot_emg_data(csv_file_path)
