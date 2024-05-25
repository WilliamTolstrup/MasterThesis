import pandas as pd
import numpy as np
from sklearn.model_selection import train_test_split, GridSearchCV
from sklearn.preprocessing import StandardScaler
from sklearn.discriminant_analysis import LinearDiscriminantAnalysis as LDA
from sklearn.svm import SVC
from sklearn.metrics import accuracy_score, confusion_matrix, classification_report
import matplotlib.pyplot as plt
import joblib


# Load the dataset
#df = pd.read_csv('features.csv')
df = pd.read_csv('data_file.csv')


# Select features and target variable
# X = df[['combined_contraction',
#         'acc_y_derivative',
#         'elbow_angle']]

X = df[[#'ch1_contraction',
        #'ch2_contraction',
        'combined_contraction',
        'emg_envelope_ch1',
        'emg_envelope_ch2',
        #'ln_acc_y',
        'acc_y_derivative',
      #  'elbow_angle'


]]

y = df['state']

# Split the dataset into training and test sets
X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42)

# Standardize features by removing the mean and scaling to unit variance
scaler = StandardScaler()
X_train_scaled = scaler.fit_transform(X_train)
X_test_scaled = scaler.transform(X_test)

joblib.dump(scaler, 'scaler.pk1') # Save scaler

# Define the parameter grid for SVM
param_grid = {'C': [0.1, 1, 2, 5, 10],
              'gamma': [0.1, 0.01, 0.001],
              'kernel': ['linear', 'poly', 'sigmoid']}

# Perform cross-validation and hyperparameter tuning
svm_grid = GridSearchCV(SVC(), param_grid, cv=5, scoring='accuracy', verbose=1, n_jobs=-1)
svm_grid.fit(X_train_scaled, y_train)

# Get the best parameters and refit the model
best_params = svm_grid.best_params_
best_svm = svm_grid.best_estimator_
best_svm.fit(X_train_scaled, y_train)

# Print the best parameters including the selected kernel
print("Best SVM Parameters:")
print(best_params)

# Save the SVM model when satisfied. 
joblib.dump(best_svm, 'svm_model.pk1')

# Predictions and Evaluations
y_pred_best_svm = best_svm.predict(X_test_scaled)
print("Best SVM Accuracy:", accuracy_score(y_test, y_pred_best_svm))
print("\nBest SVM Classification Report:\n", classification_report(y_test, y_pred_best_svm))

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

# Plot confusion matrix for the best SVM model
plt.figure(figsize=(6, 6))
plot_confusion_matrix(confusion_matrix(y_test, y_pred_best_svm), best_svm.classes_, 'Best SVM Confusion Matrix')
plt.show()
