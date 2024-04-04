import pandas as pd
import numpy as np
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler
from sklearn.discriminant_analysis import LinearDiscriminantAnalysis as LDA
from sklearn.svm import SVC
from sklearn.metrics import accuracy_score, confusion_matrix, classification_report
import matplotlib.pyplot as plt

# Load the dataset
df = pd.read_csv('data_file.csv')

# Functions to calculate features
def mean_absolute_value(signal):
    return np.mean(np.abs(signal))

def zero_crossings(signal):
    return ((signal[:-1] * signal[1:]) < 0).sum()

# Calculate EMG features for each row
df['emg_mav_ch1'] = df['emg_raw_ch1'].apply(mean_absolute_value)
df['emg_mav_ch2'] = df['emg_raw_ch2'].apply(mean_absolute_value)
df['emg_zc_ch1'] = df['emg_raw_ch1'].apply(zero_crossings)
df['emg_zc_ch2'] = df['emg_raw_ch2'].apply(zero_crossings)

# Calculate EMG features on filtered data for each row
df['emg_mav_ch1_filtered'] = df['emg_filtered_ch1'].apply(mean_absolute_value)
df['emg_mav_ch2_filtered'] = df['emg_filtered_ch2'].apply(mean_absolute_value)
df['emg_zc_ch1_filtered'] = df['emg_filtered_ch1'].apply(zero_crossings)
df['emg_zc_ch2_filtered'] = df['emg_filtered_ch2'].apply(zero_crossings)

# Calculate magnitude for accelerometer
accel_cols = ['accelerometer_x', 'accelerometer_y', 'accelerometer_z']
df['acc_mag'] = df.apply(lambda row: np.sqrt(sum(row[col]**2 for col in accel_cols)), axis=1)

# Select features and target variable
X = df[['emg_mav_ch1', 'emg_mav_ch2', 'emg_zc_ch1', 'emg_zc_ch2', 
        'emg_mav_ch1_filtered', 'emg_mav_ch2_filtered', 'emg_zc_ch1_filtered', 'emg_zc_ch2_filtered',
        'acc_mag']]
y = df['state']

# Split the dataset into training and test sets
X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42)

# Standardize features by removing the mean and scaling to unit variance
scaler = StandardScaler()
X_train_scaled = scaler.fit_transform(X_train)
X_test_scaled = scaler.transform(X_test)

# Train an LDA model
lda = LDA()
lda.fit(X_train_scaled, y_train)

# Train an SVM model
svm = SVC(kernel='linear')
svm.fit(X_train_scaled, y_train)

# Predictions and Evaluations
y_pred_lda = lda.predict(X_test_scaled)
y_pred_svm = svm.predict(X_test_scaled)
print("LDA Accuracy:", accuracy_score(y_test, y_pred_lda))
print("SVM Accuracy:", accuracy_score(y_test, y_pred_svm))
print("\nLDA Classification Report:\n", classification_report(y_test, y_pred_lda))
print("\nSVM Classification Report:\n", classification_report(y_test, y_pred_svm))

# Plot confusion matrices
def plot_confusion_matrix(cm, classes, title):
    plt.imshow(cm, interpolation='nearest', cmap=plt.cm.Blues)
    plt.title(title)
    plt.colorbar()
    tick_marks = np.arange(len(classes))
    plt.xticks(tick_marks, classes, rotation=45)
    plt.yticks(tick_marks, classes)
    plt.tight_layout()
    plt.ylabel('True label')
    plt.xlabel('Predicted label')

plt.figure(figsize=(12, 6))
plt.subplot(1, 2, 1)
plot_confusion_matrix(confusion_matrix(y_test, y_pred_lda), lda.classes_, 'LDA Confusion Matrix')
plt.subplot(1, 2, 2)
plot_confusion_matrix(confusion_matrix(y_test, y_pred_svm), svm.classes_, 'SVM Confusion Matrix')
plt.show()
