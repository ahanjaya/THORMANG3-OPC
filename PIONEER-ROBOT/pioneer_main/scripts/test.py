#!/usr/bin/env python3

import rospy
import rospkg
import keras
import numpy as np
import tensorflow as tf
from keras.models import load_model
from keras.models import Sequential
from keras.layers import Conv3D, MaxPooling3D, LeakyReLU
from keras.utils.np_utils import to_categorical
from sklearn.metrics import classification_report
from sklearn.model_selection import train_test_split
from keras.layers.core import Dense, Dropout, Activation, Flatten

class Deep_Learning:
    def __init__(self):
        rospy.init_node('pioneer_cross_deep_learning')
        rospy.loginfo("[DL] Pioneer Deep Learning Cross Arm- Running")

        rospack           = rospkg.RosPack()
        self.pcl_dataset  = rospack.get_path("pioneer_main") + "/data/cross_arm/cross_arm_dataset.npz"
        self.pcl_model    = rospack.get_path("pioneer_main") + "/data/cross_arm/cross_arm_model.h5"
        self.number_class = 2
        self.load_model   = True

    def load_data(self, path):
        datasets    = np.load(path)
        self.data   = datasets['data']
        self.labels = datasets['labels']

        # reshaping data
        self.data = self.data.reshape(self.data.shape[0], 32, 32, 32, 1)
        # one hot encoded
        self.labels = to_categorical(self.labels)

        rospy.loginfo('[DL] Total data : {}, {}'.format(self.data.shape,   type(self.data)))
        rospy.loginfo('[DL] Total label: {}, {}'.format(self.labels.shape, type(self.labels)))

        X = self.data
        y = self.labels
        x_train, x_test, y_train, y_test = train_test_split(X, y, test_size=0.33, random_state=42)
        rospy.loginfo('[DL] Train Data : {}, {}'.format(x_train.shape, y_train.shape))
        rospy.loginfo('[DL] Test  Data : {}, {}'.format(x_test.shape,  y_test.shape))

        return (x_train, y_train), (x_test, y_test)

    

    def train(self, train_data, test_data):
        model = Sequential()
        model.add(Conv3D(32, input_shape=(32, 32, 32, 1), kernel_size=(5, 5, 5), strides=(2, 2, 2), data_format='channels_last'))
        model.add(LeakyReLU(alpha=0.1))
        model.add(Conv3D(32, kernel_size=(3, 3, 3), strides=(1, 1, 1), data_format='channels_last'))
        model.add(LeakyReLU(alpha=0.1))
        model.add(MaxPooling3D(pool_size=(2, 2, 2), data_format='channels_last',))
        model.add(Flatten())
        model.add(Dense(128, activation='linear'))
        model.add(Dense(units=self.number_class, activation='softmax'))
        model.summary()

        # model.compile(loss='binary_crossentropy', optimizer='adam', metrics=['accuracy'])
        model.compile(loss='categorical_crossentropy', optimizer='adam', metrics=['accuracy'])
        # print(model.metrics_names)

        batch_size = 1
        epochs     = 10
        x_train, y_train = train_data
        x_test,  y_test  = test_data

        model.fit(x_train, y_train, batch_size=batch_size, epochs=epochs, verbose=1, validation_data=(x_test, y_test))
        return model

    def evaluate(self, model, test_data):
        x_test, y_test = test_data
        score = model.evaluate(x_test, y_test, verbose=0)
        rospy.loginfo('[DL] Test loss : {}, {}'.format(score[0], score[1]))

        predictions = model.predict_classes(x_test)
        y_test      = np.argmax(y_test, axis=1)
        report      = classification_report(y_test, predictions)
        print(report)

    def save(self, model):
        model.save(self.pcl_model)
        rospy.loginfo('[DL] Saved model: {}'.format(self.pcl_model))

    def prediction(self, model, x):
        return model.predict_classes(x)

    def run(self):
        rospy.loginfo('[DL] Load model: {}'.format(self.load_model))

        (x_train, y_train), (x_test, y_test) = self.load_data(self.pcl_dataset)

        if not self.load_model:
            model = self.train((x_train, y_train), (x_test, y_test))
            self.save(model)
            self.evaluate(model, (x_test, y_test))
        else:
            model = load_model(self.pcl_model)
            self.evaluate(model, (x_test, y_test))

        # pred = self.prediction(model, x_train)
        # print(pred)
        # print(np.argmax(y_train, axis=1))
        
if __name__ == '__main__':
    dl = Deep_Learning()
    dl.run()


    # def random_rotation(self, volume, rotation):
    #     theta_z      = np.radians(rotation)
    #     rot_matrix_z = np.array([ [np.cos(theta_z), -np.sin(theta_z), 0, 0],
    #                               [np.sin(theta_z), np.cos(theta_z), 0, 0],
    #                               [0, 0, 1, 0],
    #                               [0, 0, 0, 1]] )
    #     # channel_images = [ affine_transform(x, rot_matrix_z) for x in volume ]
    #     channel_images = [affine_transform(x, rot_matrix_z, final_offset, order=0, mode='nearest', cval=cval) for x in volume]
    #     volume_rotated = np.stack(channel_images, axis=0)
    #     # volume_rotated = affine_transform(volume, rot_matrix_z, mode='nearest')

    #     # rotated = self.random_rotation(data[0], 20)
    #     # print(rotated.shape)

    #     # rotated = self.rotateZ(20) * data[0]
    #     # rotated = np.rot90(data[0], 2)
    #     # rotated = rotate(data[0], angle=45)

    #     # fig1 = plt.figure()
    #     # ax1 = fig1.gca(projection='3d')
    #     # ax1.voxels(rotated)
    #     return volume_rotated 