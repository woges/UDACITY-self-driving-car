import csv
import cv2
import numpy as np
import sklearn
import os
import matplotlib.pyplot as plt

from sklearn.model_selection import train_test_split
from sklearn.utils import shuffle

from keras.models import Sequential, Model
from keras.layers import Flatten, Dense, Activation, Dropout, Lambda
from keras.layers.convolutional import Convolution2D, MaxPooling2D
from keras.layers.normalization import BatchNormalization
from keras.optimizers import Adam
from keras.callbacks import TensorBoard, ModelCheckpoint, EarlyStopping


#### Start hyperparameters ############################################
NB_EPOCHS = 20
BATCH_SIZE = 64
DROPOUT = 0.6
LEARNING_RATE=6e-4
MEAS_CORR=0.35
#### End hyperparameters ##############################################

def crop_image(img, img_height=75, img_width=200):
    height = img.shape[1]
    width = img.shape[2]
    y_start = 60
    return img[:,y_start:y_start+img_height, 0:width,: ]

# https://keras.io/callbacks/#usage-of-callbacks
def get_callbacks():
    earlystopping = EarlyStopping(monitor='val_loss', min_delta=0,
                                  patience=1, verbose=1, mode='auto')
    return [earlystopping]

#https://chatbotslife.com/using-augmentation-to-mimic-human-driving-496b569760a9#.8ipx8fj4q
def augment_brightness_camera_images(image):
    image1 = cv2.cvtColor(image,cv2.COLOR_RGB2HSV)
    image1 = np.array(image1, dtype = np.float64)
    random_bright = .5+np.random.uniform()
    image1[:,:,2] = image1[:,:,2]*random_bright
    image1[:,:,2][image1[:,:,2]>255]  = 255
    image1 = np.array(image1, dtype = np.uint8)
    image1 = cv2.cvtColor(image1,cv2.COLOR_HSV2RGB)
    return image1

#### Start Data reading ###############################################
lines = []
with open('../data_run_test_001/driving_log.csv') as csvfile:
    reader = csv.reader(csvfile)
    for line in reader:
        lines.append(line)

img_path = []
measurements =  []

for line in lines:
    center_path = line[0]
    left_path = line[1]
    right_path = line[2]
    img_path.append(center_path)
    img_path.append(left_path)
    img_path.append(right_path)
    center_measurement = float(line[3])
    left_measurement = float(line[3]) + MEAS_CORR + 0.05
    right_measurement = float(line[3]) - MEAS_CORR
    measurements.append(center_measurement)
    measurements.append(left_measurement)
    measurements.append(right_measurement)
#### End Data reading ###############################################

#### Start Data splitting ###########################################
train_img, validation_img, train_meas, validation_meas=train_test_split(img_path, measurements , test_size=0.2)
#### End Data splitting #############################################

##### Start generator function ########################################
def generator(img_path, img_meas, batch_size=BATCH_SIZE):
    num_samples = len(img_path)
    k = 0
    while 1:
        shuffle(img_path, img_meas)
        k = 0
        for offset in range(0, num_samples, batch_size):
            k+=1
            if num_samples-offset-1 < batch_size:
                end=num_samples
            else:
                end=offset+batch_size

            batch_img = img_path[offset:end]
            batch_meas = img_meas[offset:end]

            images=[]
            measurements=[]
            for path, meas in zip(batch_img, batch_meas):

                image = cv2.imread(path)
                images.append(image)
                measurements.append(meas)
            #### Start augmentation ##################################
                augmented_images, augmented_measurements = [],[]
                for image,measurement in zip(images, measurements):
                    augmented_images.append(image)
                    augmented_measurements.append(measurement)
                    augmented_images.append(cv2.flip(image,1))
                    augmented_measurements.append(measurement*-1.0)
#                    augmented_images.append(augment_brightness_camera_images(image))
#                    augmented_measurements.append(measurement)                   
            #### End  augmentation ##################################
            X_train = np.array(augmented_images)
            #### Start image trim ####################################
            X_train = crop_image(X_train)
            #### End image trim ######################################
            y_train = np.array(augmented_measurements)
            yield shuffle(X_train, y_train)
##### End generator function ########################################


train_generator = generator(train_img, train_meas, batch_size=BATCH_SIZE)
#print('validation generator')
validation_generator = generator(validation_img, validation_meas, batch_size=BATCH_SIZE)
#### End Data reading and preprocessing #############################

#### start model parameter ##########################################
INPUT_SHAPE = (75,320,3)
# number of convolutional filters to use
NB_FILTERS_1 = 32
NB_FILTERS_2 = 46
NB_FILTERS_3 = 64
NB_FILTERS_4 = 80
NB_FILTERS_5 = 100
# convolution kernel size
KERNEL_SIZE_1 = (5, 5)
KERNEL_SIZE_2 = (3, 3)
# convolution strides
STRIDES_1 = (2,2)
STRIDES_2 = (1,1)
# padding for cnn2d
PADD_CNN2D = 'valid'
# size of pooling area for max pooling
POOL_SIZE = (2, 2)
#### end model parameter ############################################

# https://images.nvidia.com/content/tegra/automotive/images/2016/solutions/pdf/end-to-end-dl-using-px.pdf
#### Start Model ####################################################
model = Sequential()
## Preprocessing
#model.add(Lambda(lambda x: (x / 255.0) - 0.5, input_shape=(160,320,3)))
model.add(BatchNormalization(input_shape=(75,320,3), mode=0,axis=3))
model.add(Convolution2D(NB_FILTERS_1, KERNEL_SIZE_1[0], KERNEL_SIZE_1[1], 
                        subsample=STRIDES_1,border_mode=PADD_CNN2D,
                        input_shape=INPUT_SHAPE,
                        activation="relu"))

model.add(Convolution2D(NB_FILTERS_2, KERNEL_SIZE_1[0], KERNEL_SIZE_1[1], 
                        subsample=STRIDES_1,border_mode=PADD_CNN2D,
                        input_shape=INPUT_SHAPE,
                        activation="relu"))

model.add(Convolution2D(NB_FILTERS_3, KERNEL_SIZE_1[0], KERNEL_SIZE_1[1], 
                        subsample=STRIDES_1,border_mode=PADD_CNN2D,
                        input_shape=INPUT_SHAPE,
                        activation="relu"))

model.add(Convolution2D(NB_FILTERS_4, KERNEL_SIZE_2[0], KERNEL_SIZE_2[1], 
                        subsample=STRIDES_2,border_mode=PADD_CNN2D,
                        input_shape=INPUT_SHAPE,
                        activation="relu"))

model.add(Convolution2D(NB_FILTERS_5, KERNEL_SIZE_2[0], KERNEL_SIZE_2[1], 
                        subsample=STRIDES_2,border_mode=PADD_CNN2D,
                        input_shape=INPUT_SHAPE,
                        activation="relu"))

model.add(Flatten(input_shape = INPUT_SHAPE))
model.add(Dense(240))
model.add(Dropout(DROPOUT))
model.add(Dense(60))
model.add(Dropout(DROPOUT))
model.add(Dense(15))
model.add(Dropout(DROPOUT))
model.add(Dense(1))

#### End Model ######################################################

#### Start Model Training ###########################################

model.compile(optimizer=Adam(LEARNING_RATE), loss="mse", )

print('Training samples: ', len(train_img))
history_obj = model.fit_generator(train_generator, samples_per_epoch=(2*len(train_img)),\
                    callbacks=get_callbacks(),validation_data=validation_generator,\
                    nb_val_samples=(2*len(validation_img)), nb_epoch=NB_EPOCHS, verbose=2)
#### End Model Training #############################################
#### Start Model Loss History #######################################

plt.plot(history_obj.history['loss'])
plt.plot(history_obj.history['val_loss'])
plt.title('model mean squared error loss')
plt.ylabel('mean squared error loss')
plt.xlabel('epoch')
plt.legend(['training set', 'validation set'], loc='upper right')
plt.show()

#### End Model Loss History #########################################
#### Start Model Saving #############################################
model.save('model.h5')

#### End Model Saving ###############################################
  
