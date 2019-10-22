#!/usr/bin/env python3

import rospy
import rospkg
import keras
import numpy as np
import tensorflow as tf
from keras.models import load_model
from keras.models import Sequential
from keras.utils.np_utils import to_categorical
from sklearn.metrics import classification_report
from sklearn.model_selection import train_test_split
from keras.layers import Conv3D, MaxPooling3D, LeakyReLU
from keras.layers.core import Dense, Dropout, Activation, Flatten

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.ndimage.interpolation import affine_transform
import pptk

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
        datasets = np.load(path)
        data     = datasets['data']
        labels   = datasets['labels']
        return data, labels

    def preprocess_data(self, data, labels):
        data   = data.reshape(data.shape[0], 32, 32, 32, 1) # reshaping data
        labels = to_categorical(labels)                     # one hot encoded
        rospy.loginfo('[DL] Total data : {}, {}'.format(data.shape,   type(data)))
        rospy.loginfo('[DL] Total label: {}, {}'.format(labels.shape, type(labels)))

        X = data
        y = labels
        x_train, x_test, y_train, y_test = train_test_split(X, y, test_size=0.33, random_state=42)
        rospy.loginfo('[DL] Train Data : {}, {}'.format(x_train.shape, y_train.shape))
        rospy.loginfo('[DL] Test  Data : {}, {}'.format(x_test.shape,  y_test.shape))

        return (x_train, y_train), (x_test, y_test)

    def plot_3d(self, ax, x, y, z, title=""):
        ax.scatter(x, y, z, c='green')

        ax.set_xlabel('x')
        ax.set_ylabel('y')
        ax.set_zlabel('z')

        ax.set_xlim(0, 32)
        ax.set_ylim(0, 32)
        ax.set_zlim(0, 32)

        ax.set_title(title)

    def rotation_pts(self, point, theta):
        theta = np.radians(theta)
        rot_z = np.array([ [np.cos(theta), -np.sin(theta), 0],
                           [np.sin(theta),  np.cos(theta), 0],
                           [0,              0,             1] ])
        # return rot_z.dot(point)
        return point.dot(rot_z)
        
    def augmentation_data(self, data, labels):
        # fig = plt.figure()
        # ax  = fig.gca(projection='3d')
        # ax.voxels(data[0])

        d = data[0]
        x, y, z = d.nonzero()

        # print(x,y,z)
        # print(x.shape, y.shape, z.shape)

        vox = np.stack((x, y, z), axis=1)
        a = pptk.viewer(vox)

        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        self.plot_3d(ax, x, y, z,)

        rotated = [ self.rotation_pts(i, -20) for i in vox]
        rotated = np.array(rotated)
        b = pptk.viewer(rotated)

        x, y, z = rotated[:,0], rotated[:,1], rotated[:,2]
        fig1 = plt.figure()
        ax1 = fig1.add_subplot(111, projection='3d')
        self.plot_3d(ax1, x, y, z,)
               

        plt.show(block=False)
        input('[Close]')

        a.close()
        b.close()
        

        return data, labels

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

        data, labels = self.load_data(self.pcl_dataset)
        data, labels = self.augmentation_data(data, labels)

        # (x_train, y_train), (x_test, y_test) = self.preprocess_data(data, labels)

        # if not self.load_model:
        #     model = self.train((x_train, y_train), (x_test, y_test))
        #     self.save(model)
        #     self.evaluate(model, (x_test, y_test))
        # else:
        #     model = load_model(self.pcl_model)
        #     self.evaluate(model, (x_test, y_test))

        # pred = self.prediction(model, x_train)
        # print(pred)
        # print(np.argmax(y_train, axis=1))
        
if __name__ == '__main__':
    dl = Deep_Learning()
    dl.run()