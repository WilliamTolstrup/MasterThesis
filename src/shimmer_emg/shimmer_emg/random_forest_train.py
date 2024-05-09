import pandas as pd
from sklearn.model_selection import train_test_split, GridSearchCV, cross_val_score
from sklearn.ensemble import RandomForestClassifier
from sklearn.preprocessing import LabelEncoder
from sklearn.metrics import accuracy_score
import matplotlib.pyplot as plt
import numpy as np

# Load the dataset
df = pd.read_csv('data_file.csv')

# Select the desired feature columns
feature_cols = ['combined_contraction',
                'acc_y_derivative',
                'elbow_angle']

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
