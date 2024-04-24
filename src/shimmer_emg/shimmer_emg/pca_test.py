from sklearn.decomposition import PCA
from sklearn.preprocessing import StandardScaler
import pandas as pd

# Load the dataset
data = pd.read_csv('data_file.csv')

data = data[['emg_raw_mav_ch1', 'emg_raw_mav_ch2', 
             'emg_raw_rms_ch1', 'emg_raw_rms_ch2', 
             'emg_raw_sd_ch1', 'emg_raw_sd_ch2', 
             'emg_raw_wl_ch1', 'emg_raw_wl_ch2', 
             'raw_coeff_1', 'raw_coeff_2', 'raw_coeff_3', 'raw_coeff_4', 'raw_coeff_5', 'raw_coeff_6', 'raw_coeff_7', 'raw_coeff_8', 
             'emg_filtered_mav_ch1', 'emg_filtered_mav_ch2', 
             'emg_filtered_rms_ch1', 'emg_filtered_rms_ch2', 
             'emg_filtered_sd_ch1', 'emg_filtered_sd_ch2', 
             'emg_filtered_wl_ch1', 'emg_filtered_wl_ch2', 
             'filtered_coeff_1', 'filtered_coeff_2', 'filtered_coeff_3', 'filtered_coeff_4', 'filtered_coeff_5', 'filtered_coeff_6', 'filtered_coeff_7', 'filtered_coeff_8', 
             'acc_mav_x', 'acc_mav_y', 'acc_mav_z', 
             'acc_rms_x', 'acc_rms_y', 'acc_rms_z',
             'acc_sd_x', 'acc_sd_y', 'acc_sd_z',]]

# Assuming 'data' is your DataFrame with all features including filtered EMG and accelerometer data
features = data.columns.tolist()
x = data.loc[:, features].values

# Standardizing the features
x = StandardScaler().fit_transform(x)

# PCA
pca = PCA(n_components=2)  # You can choose how many components to retain
principalComponents = pca.fit_transform(x)
principalDf = pd.DataFrame(data=principalComponents, columns=['PC 1', 'PC 2'])

# Explained variance ratio
print(pca.explained_variance_ratio_)
# This will give you the percentage of variance explained by each of the selected components.

# Look at PCA components to see which features contribute most
print(abs(pca.components_))
