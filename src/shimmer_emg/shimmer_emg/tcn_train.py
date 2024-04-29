import pandas as pd
from sklearn.model_selection import train_test_split, GridSearchCV
from sklearn.preprocessing import StandardScaler, LabelEncoder
import tensorflow as tf
from keras import Sequential, layers, utils, callbacks, regularizers, models, optimizers
from scikeras.wrappers import KerasClassifier
from tcn import TCN, tcn_full_summary

# Load the dataset
df = pd.read_csv('data_file.csv')

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

# Encode the classification labels to integers and convert to categorical
encoder = LabelEncoder()
y_encoded = encoder.fit_transform(y)
y_categorical = utils.to_categorical(y_encoded)

# Split the dataset into training and test sets
X_train, X_test, y_train, y_test = train_test_split(X, y_categorical, test_size=0.2, random_state=42)

# Reshape data for TCN input (batch_size, timesteps, features)
X_train = X_train.reshape((X_train.shape[0], 1, X_train.shape[1]))
X_test = X_test.reshape((X_test.shape[0], 1, X_test.shape[1]))

def build_tcn_model(input_shape, num_classes):
    inputs = layers.Input(shape=input_shape)
    tcn_layer = TCN(
        nb_filters=64,
        kernel_size=3,
        nb_stacks=2,
        dilations=[1, 2, 4, 8],
        padding='causal',
        use_skip_connections=True,
        dropout_rate=0.2,
        return_sequences=False
    )(inputs)
    outputs = layers.Dense(num_classes, activation='softmax')(tcn_layer)
    model = models.Model(inputs=inputs, outputs=outputs)
    model.compile(optimizer=optimizers.Adam(learning_rate=0.002),
                  loss='categorical_crossentropy',
                  metrics=['accuracy'])
    return model

# Build the TCN model
input_shape = (X_train.shape[1], X_train.shape[2])  # (timesteps, features)
num_classes = y_train.shape[1]
model = build_tcn_model(input_shape, num_classes)

# Model summary
tcn_full_summary(model, expand_residual_blocks=False)

# Train the model
history = model.fit(X_train, y_train, epochs=50, validation_split=0.1, batch_size=32)

# Evaluate the model
loss, accuracy = model.evaluate(X_test, y_test)
print(f"Test accuracy: {accuracy * 100:.2f}%")