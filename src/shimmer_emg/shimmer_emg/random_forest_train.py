import pandas as pd
from sklearn.model_selection import train_test_split, GridSearchCV, cross_val_score
from sklearn.ensemble import RandomForestClassifier
from sklearn.preprocessing import LabelEncoder
from sklearn.metrics import accuracy_score
import matplotlib.pyplot as plt
import numpy as np

# Load the dataset
df = pd.read_csv('features_file.csv')

# Select the desired feature columns
feature_cols = ['emg_raw_mav_ch1', 'emg_raw_mav_ch2', 
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
                'acc_sd_x', 'acc_sd_y', 'acc_sd_z']

X = df[feature_cols].values
y = df['state'].values

# Encode the classification labels to integers
encoder = LabelEncoder()
y_encoded = encoder.fit_transform(y)

# Define the model
rf = RandomForestClassifier(random_state=42)

# Define a parameter grid to search over
param_grid = {
    'n_estimators': [25, 50, 100, 200],
    'max_depth': [5, 10, 20, 30, None],
    'min_samples_split': [2, 5, 10, 20],
    'min_samples_leaf': [1, 2, 4, 8]
}

# Set up GridSearchCV
grid_search = GridSearchCV(estimator=rf, param_grid=param_grid, cv=5, scoring='accuracy', verbose=1, n_jobs=-1)
grid_search.fit(X, y_encoded)

# Get the best parameters
print("Best parameters:", grid_search.best_params_)

# Evaluate the best model using cross-validation
best_rf = grid_search.best_estimator_
cv_scores = cross_val_score(best_rf, X, y_encoded, cv=5, scoring='accuracy')
print(f"CV Accuracy Scores: {cv_scores}")
print(f"Mean CV Accuracy: {cv_scores.mean():.2f}")

# Feature Importance
importances = best_rf.feature_importances_
indices = np.argsort(importances)[::-1]

# Plot Feature Importance
plt.figure(figsize=(12, 6))
plt.title('Feature Importances')
plt.bar(range(len(indices)), importances[indices], color='r', align='center')
plt.xticks(range(len(indices)), [feature_cols[i] for i in indices], rotation=90)
plt.xlim([-1, len(indices)])
plt.show()
