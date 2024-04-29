import pandas as pd
from sklearn.model_selection import train_test_split, GridSearchCV
from sklearn.preprocessing import StandardScaler, LabelEncoder
import tensorflow as tf
from keras import Sequential, layers, utils, callbacks, regularizers, models
from scikeras.wrappers import KerasClassifier

# Load the dataset
df = pd.read_csv('data_file.csv')

# Select the desired feature columns based on provided names
feature_cols = ['emg_raw_mav_ch1', 'emg_raw_mav_ch2', 
                'emg_raw_rms_ch1', 'emg_raw_rms_ch2',
                'emg_raw_sd_ch1', 'emg_raw_sd_ch2', 
                #'emg_raw_wl_ch1', 'emg_raw_wl_ch2',
                'raw_coeff_1', 'raw_coeff_2', 'raw_coeff_3', 'raw_coeff_4', 'raw_coeff_5', 'raw_coeff_6', 'raw_coeff_7', 'raw_coeff_8',
                'emg_filtered_mav_ch1', 'emg_filtered_mav_ch2', 
                'emg_filtered_rms_ch1', 'emg_filtered_rms_ch2', 
                'emg_filtered_sd_ch1', 'emg_filtered_sd_ch2',
                #'emg_filtered_wl_ch1', 'emg_filtered_wl_ch2', 
                'filtered_coeff_1', 'filtered_coeff_2', 'filtered_coeff_3', 'filtered_coeff_4', 'filtered_coeff_5', 'filtered_coeff_6', 'filtered_coeff_7', 'filtered_coeff_8', 
                'acc_mav_x', 'acc_mav_y', 'acc_mav_z',
                'acc_rms_x', 'acc_rms_y', 'acc_rms_z', 
                'acc_sd_x', 'acc_sd_y', 'acc_sd_z']

X = df[feature_cols].values
y = df['state'].values

# Encode the classification labels to integers
encoder = LabelEncoder()
encoded_Y = encoder.fit_transform(y)
# Convert integers to dummy variables (one hot encoding)
dummy_y = utils.to_categorical(encoded_Y)

# Split the dataset into training and test sets
X_train, X_test, y_train, y_test = train_test_split(X, dummy_y, test_size=0.2, random_state=42)

# Scale features
scaler = StandardScaler()
X_train = scaler.fit_transform(X_train)
X_test = scaler.transform(X_test)

# Reshape input to be [samples, time steps, features] for RNN
X_train = X_train.reshape((X_train.shape[0], 1, X_train.shape[1]))
X_test = X_test.reshape((X_test.shape[0], 1, X_test.shape[1]))

# Early stopping callback
early_stopping = callbacks.EarlyStopping(
    monitor='val_loss',
    patience=10,
    restore_best_weights=True
)

# Define the RNN model using best known parameters
def create_model():
    lstm_units = 64
    dropout_rate = 0.2
    opti = 'adam'
    l2_rate = 0.01
    model = models.Sequential()
    model.add(layers.LSTM(units=lstm_units, input_shape=(1, X_train.shape[2]), return_sequences=True))
    model.add(layers.Dropout(rate=dropout_rate))
    model.add(layers.LSTM(units=lstm_units, return_sequences=True, kernel_regularizer=regularizers.l2(l2_rate)))
    model.add(layers.Dropout(rate=dropout_rate))
    model.add(layers.LSTM(units=lstm_units))
    model.add(layers.Dropout(rate=dropout_rate))
    model.add(layers.Dense(dummy_y.shape[1], activation='softmax'))
    model.compile(loss='categorical_crossentropy', optimizer=opti, metrics=['accuracy'])
    return model

# Different params:
    # 'lstm_units': [32, 64, 128],
    # 'dropout_rate': [0.2, 0.5, 0.7],
    # 'optimizer': ['adam', 'rmsprop'],
    # 'l2_rate': [0.01, 0.001, 0.0001],
# lstm_units   dropout_rate   optimizer   l2_rate   accuracy%

#     32            0.2         'adam'     0.01       78.5%
#     32            0.2         'adam'     0.001      77.0%
#     32            0.2         'adam'     0.0001     73.3%
#     32            0.2      'rmsprop'     0.01       73.9%
#     32            0.2      'rmsprop'     0.001      74.8%
#     32            0.2      'rmsprop'     0.0001     76.0%

#     32            0.5         'adam'     0.01       64.1%
#     32            0.5         'adam'     0.001      64.4%
#     32            0.5         'adam'     0.0001     63.0%
#     32            0.5      'rmsprop'     0.01       62.9%
#     32            0.5      'rmsprop'     0.001      64.7%
#     32            0.5      'rmsprop'     0.0001     63.7%

#     32            0.7         'adam'     0.01       59.3%
#     32            0.7         'adam'     0.001      62.6%
#     32            0.7         'adam'     0.0001     61.9%
#     32            0.7      'rmsprop'     0.01       53.2%
#     32            0.7      'rmsprop'     0.001      61.4%
#     32            0.7      'rmsprop'     0.0001     61.9%


#     64            0.2         'adam'     0.01       77.5%
#     64            0.2         'adam'     0.001      77.9%
#     64            0.2         'adam'     0.0001     78.5%
#     64            0.2      'rmsprop'     0.01       77.1%
#     64            0.2      'rmsprop'     0.001      77.4%
#     64            0.2      'rmsprop'     0.0001     77.1%

#     64            0.5         'adam'     0.01       64.5%
#     64            0.5         'adam'     0.001      74.0%
#     64            0.5         'adam'     0.0001     73.2%
#     64            0.5      'rmsprop'     0.01       63.4%
#     64            0.5      'rmsprop'     0.001      75.3%
#     64            0.5      'rmsprop'     0.0001     77.6%

#     64            0.7         'adam'     0.01       62.9%
#     64            0.7         'adam'     0.001      62.8%
#     64            0.7         'adam'     0.0001     64.1%
#     64            0.7      'rmsprop'     0.01       64.0%
#     64            0.7      'rmsprop'     0.001      64.5%
#     64            0.7      'rmsprop'     0.0001     64.6%


#    128            0.2         'adam'     0.01       75.0%
#    128            0.2         'adam'     0.001      76.4%
#    128            0.2         'adam'     0.0001     78.7%
#    128            0.2      'rmsprop'     0.01       75.8%
#    128            0.2      'rmsprop'     0.001      76.4%
#    128            0.2      'rmsprop'     0.0001     75.1%

#    128            0.5         'adam'     0.01       75.2%
#    128            0.5         'adam'     0.001      76.9%
#    128            0.5         'adam'     0.0001     78.8%
#    128            0.5      'rmsprop'     0.01       75.8%
#    128            0.5      'rmsprop'     0.001      76.3%
#    128            0.5      'rmsprop'     0.0001     76.4%

#    128            0.7         'adam'     0.01       64.6%
#    128            0.7         'adam'     0.001      70.3%
#    128            0.7         'adam'     0.0001     74.0%
#    128            0.7      'rmsprop'     0.01       64.0%
#    128            0.7      'rmsprop'     0.001      65.3%
#    128            0.7      'rmsprop'     0.0001     74.5%
    
# Instantiate the model
model = create_model()

# Train the model
model.fit(X_train, y_train, epochs=160, batch_size=32, validation_split=0.1, callbacks=[early_stopping])

# Evaluate the model on the test data
scores = model.evaluate(X_test, y_test, verbose=0)
print(f"Test Accuracy: {scores[1] * 100}%")