import pandas as pd
import numpy as np
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler
from sklearn.discriminant_analysis import LinearDiscriminantAnalysis as LDA
from sklearn.svm import SVC
from sklearn.metrics import accuracy_score, confusion_matrix, classification_report
import matplotlib.pyplot as plt
import joblib

### TODO:
# Test with and without raw emg features/ coefficients.
# Also, figure out what to do with the imu data; should it be part of the classifier or solely for angle estimation?


# Load the dataset
df = pd.read_csv('data_file.csv')


# Select features and target variable
X = df[['emg_raw_mav_ch1', 'emg_raw_mav_ch2', 
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
y = df['state']

# Split the dataset into training and test sets
X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42)

# Standardize features by removing the mean and scaling to unit variance
scaler = StandardScaler()
X_train_scaled = scaler.fit_transform(X_train)
X_test_scaled = scaler.transform(X_test)

joblib.dump(scaler, 'scaler.pk1') # Save scaler

# Train LDA model
lda = LDA()
lda.fit(X_train_scaled, y_train)

# Train SVM model
svm = SVC(kernel='linear')
svm.fit(X_train_scaled, y_train)

# Save the SVM model when satisfied. 
joblib.dump(svm, 'svm_model.pk1')

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
