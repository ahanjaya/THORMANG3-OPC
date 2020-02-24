#!/usr/bin/env python3

import keras
import rospy
import rospkg
import itertools
import numpy as np
import tensorflow as tf
import matplotlib.pyplot as plt

from keras.utils import plot_model
from keras.optimizers import Adam, SGD
from keras.callbacks import ModelCheckpoint
from keras.utils.np_utils import to_categorical
from keras.layers import Conv2D, Conv3D, MaxPooling2D, MaxPooling3D, LeakyReLU
from keras.models import load_model, Sequential, model_from_json, Model
from keras.layers.normalization import BatchNormalization
from keras.layers.core import Dense, Dropout, Activation, Flatten

from sklearn.preprocessing import LabelEncoder
from sklearn.model_selection import StratifiedKFold
from sklearn.model_selection import train_test_split
from sklearn.metrics import confusion_matrix, classification_report, accuracy_score

class Deep_Learning:
    def __init__(self):
        rospy.init_node('pioneer_cross_deep_learning')
        rospy.loginfo("[DL] Pioneer Deep Learning Cross Arm- Running")

        self.plt_num    = 0
        self.batch_size = 25
        self.epochs     = 100
        self.lr         = 0.00006 # 0.0006

        rospack           = rospkg.RosPack()
        self.pcl_dataset  = rospack.get_path("pioneer_main") + "/data/cross_arm/cross_arm_dataset.npz"

        self.data_dir       = rospack.get_path("pioneer_main") + "/data/cross_arm"
        self.voxnet_model   = '{}/voxnet.json'.format(self.data_dir)
        self.voxnet_weight  = '{}/voxnet.hdf5'.format(self.data_dir)

        self.vcnn1_model    = '{}/vcnn1.json'.format(self.data_dir)
        self.vcnn1_weight   = '{}/vcnn1.hdf5'.format(self.data_dir)

        self.mvcnn1_model   = '{}/mvcnn1.json'.format(self.data_dir)
        self.mvcnn1_weight  = '{}/mvcnn1.hdf5'.format(self.data_dir)
        
        # self.growth_memory() # limit tensorflow to use all of GPU resources

    def growth_memory(self):
        physical_devices = tf.config.experimental.list_physical_devices('GPU')
        if len(physical_devices) > 0:
            for k in range(len(physical_devices)):
                tf.config.experimental.set_memory_growth(physical_devices[k], True)
                print('memory growth:', tf.config.experimental.get_memory_growth(physical_devices[k]))
        else:
            print("Not enough GPU hardware devices available")

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

    def preprocess_2D_data(self, data, labels):
        data   = data.reshape(data.shape[0], 32, 32, 32) # reshaping data
        labels = to_categorical(labels)                     # one hot encoded
        rospy.loginfo('[DL] Total 2D data : {}, {}'.format(data.shape,   type(data)))
        rospy.loginfo('[DL] Total 2D label: {}, {}'.format(labels.shape, type(labels)))

        X = data
        y = labels
        x_train, x_test, y_train, y_test = train_test_split(X, y, test_size=0.33, random_state=42)
        rospy.loginfo('[DL] Train 2D Data : {}, {}'.format(x_train.shape, y_train.shape))
        rospy.loginfo('[DL] Test  2D Data : {}, {}'.format(x_test.shape,  y_test.shape))

        return (x_train, y_train), (x_test, y_test)

    def voxnet(self, train_data, test_data, batch_size=128, epochs=50):
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
        
        # learning_rate = 0.1
        # decay_rate = learning_rate / epochs
        # momentum = 0.8
        # sgd = SGD(lr=learning_rate, momentum=momentum, decay=decay_rate, nesterov=False)
        # model.compile(loss='categorical_crossentropy', optimizer=sgd, metrics=['accuracy'])

        model.compile(loss='categorical_crossentropy', optimizer=Adam(lr=self.lr), metrics=['accuracy'])

        json_config = model.to_json()
        with open(self.voxnet_model, 'w') as json_file:
            json_file.write(json_config)

        checkpoint       = ModelCheckpoint(self.voxnet_weight, monitor='val_accuracy', verbose=1, save_best_only=True, mode='max')
        x_train, y_train = train_data
        x_test,  y_test  = test_data

        self.history = model.fit(x_train, y_train, batch_size=batch_size, epochs=epochs, verbose=1, validation_data=(x_test, y_test), callbacks=[checkpoint])
        return model

    def MV_CNN_1(self, train_data, test_data, batch_size=128, epochs=50):
        model = Sequential()
        model.add(Conv3D(32, input_shape=(32, 32, 32, 1), kernel_size=(3, 3, 3), strides=(1, 1, 1), data_format='channels_last'))
        model.add(Activation('relu'))
        model.add(MaxPooling3D(pool_size=(2, 2, 2), strides=(2, 2, 2), data_format='channels_last',))
        
        model.add(Conv3D(32, kernel_size=(3, 3, 3), strides=(1, 1, 1), data_format='channels_last'))
        model.add(Activation('relu'))

        model.add(Conv3D(32, kernel_size=(3, 3, 3), strides=(1, 1, 1), data_format='channels_last'))
        model.add(MaxPooling3D(pool_size=(2, 2, 2), strides=(2, 2, 2), data_format='channels_last',))
        model.add(Dropout(0.8))

        model.add(Flatten())
        model.add(Dense(2048, activation='linear'))
        model.add(BatchNormalization())
        model.add(Dense(units=self.number_class, activation='softmax'))
        model.summary()

        # learning_rate = 0.1
        # decay_rate = learning_rate / epochs
        # momentum = 0.8
        # sgd = SGD(lr=learning_rate, momentum=momentum, decay=decay_rate, nesterov=False)
        # model.compile(loss='categorical_crossentropy', optimizer=sgd, metrics=['accuracy'])

        model.compile(loss='categorical_crossentropy', optimizer=Adam(lr=self.lr), metrics=['accuracy'])

        json_config = model.to_json()
        with open(self.mvcnn1_model, 'w') as json_file:
            json_file.write(json_config)

        checkpoint       = ModelCheckpoint(self.mvcnn1_weight, monitor='val_accuracy', verbose=1, save_best_only=True, mode='max')
        x_train, y_train = train_data
        x_test,  y_test  = test_data

        self.history = model.fit(x_train, y_train, batch_size=batch_size, epochs=epochs, verbose=1, validation_data=(x_test, y_test), callbacks=[checkpoint])
        return model

    def V_CNN_1(self, train_data, test_data, batch_size=128, epochs=50):
        model = Sequential()
        model.add(Conv2D(64, input_shape=(32, 32, 32), kernel_size=(3, 3), strides=(1, 1), data_format='channels_last'))
        model.add(Activation('relu'))
        model.add(MaxPooling2D(pool_size=(2, 2), strides=(2, 2), data_format='channels_last',))
        
        model.add(Conv2D(64, kernel_size=(3, 3), strides=(1, 1), data_format='channels_last'))
        model.add(Activation('relu'))

        model.add(Conv2D(64, kernel_size=(3, 3), strides=(1, 1), data_format='channels_last'))
        model.add(MaxPooling2D(pool_size=(2, 2), strides=(2, 2), data_format='channels_last',))
        model.add(Dropout(0.5))

        model.add(Flatten())
        model.add(Dense(2048, activation='linear'))
        model.add(BatchNormalization())
        model.add(Dense(units=self.number_class, activation='softmax'))
        model.summary()

        # learning_rate = 0.1
        # decay_rate = learning_rate / epochs
        # momentum = 0.8
        # sgd = SGD(lr=learning_rate, momentum=momentum, decay=decay_rate, nesterov=False)
        # model.compile(loss='categorical_crossentropy', optimizer=sgd, metrics=['accuracy'])

        model.compile(loss='categorical_crossentropy', optimizer=Adam(lr=self.lr), metrics=['accuracy'])

        json_config = model.to_json()
        with open(self.vcnn1_model, 'w') as json_file:
            json_file.write(json_config)

        checkpoint       = ModelCheckpoint(self.vcnn1_weight, monitor='val_accuracy', verbose=1, save_best_only=True, mode='max')
        x_train, y_train = train_data
        x_test,  y_test  = test_data

        self.history = model.fit(x_train, y_train, batch_size=batch_size, epochs=epochs, verbose=1, validation_data=(x_test, y_test), callbacks=[checkpoint])
        return model

    def plot_single(self, mode, epochs, vox_hist, vcn1_hist, mvcn1_hist, list_vox, list_vcn1, list_mvcn1):
        a, b, c      = [], [], []
        
        self.plt_num += 1
        plt.figure(self.plt_num)

        for l in list_vox:
            v, = plt.plot(epochs, vox_hist.history[l], 'b')
            a.append(vox_hist.history[l])
        for l in list_vcn1:
            u, = plt.plot(epochs, vcn1_hist.history[l], 'g')
            b.append(vcn1_hist.history[l])
        for l in list_mvcn1:
            w, = plt.plot(epochs, mvcn1_hist.history[l], 'r')
            c.append(mvcn1_hist.history[l])
        
        if 'Loss' in mode:
            a = np.min( np.array(a) )
            b = np.min( np.array(b) )
            c = np.min( np.array(c) )
            plt.ylabel('Loss')
        else:
            a = np.max( np.array(a) )
            b = np.max( np.array(b) )
            c = np.max( np.array(c) )
            plt.ylabel('Accuracy')

        v.set_label('Voxnet - {0:.2f}'.format(a))
        u.set_label('V-CNN1 - {0:.2f}'.  format(b))
        w.set_label('CAV-CNN - {0:.2f}'. format(c))

        plt.title(mode)
        plt.xlabel('Epochs')
        plt.legend()

    def plot_history(self, vox_hist, vcn1_hist, mvcn1_hist):
        vox_loss       = [s for s in vox_hist.history.keys()   if 'loss' in s and 'val' not in s]
        vox_val_loss   = [s for s in vox_hist.history.keys()   if 'loss' in s and 'val' in s]
        vcn1_loss      = [s for s in vcn1_hist.history.keys()  if 'loss' in s and 'val' not in s]
        vcn1_val_loss  = [s for s in vcn1_hist.history.keys()  if 'loss' in s and 'val' in s]
        mvcn1_loss     = [s for s in mvcn1_hist.history.keys() if 'loss' in s and 'val' not in s]
        mvcn1_val_loss = [s for s in mvcn1_hist.history.keys() if 'loss' in s and 'val' in s]
        
        vox_acc        = [s for s in vox_hist.history.keys()   if 'acc'  in s and 'val' not in s]
        vox_val_acc    = [s for s in vox_hist.history.keys()   if 'acc'  in s and 'val' in s]
        vcn1_acc       = [s for s in vcn1_hist.history.keys()  if 'acc'  in s and 'val' not in s]
        vcn1_val_acc   = [s for s in vcn1_hist.history.keys()  if 'acc'  in s and 'val' in s]
        mvcn1_acc      = [s for s in mvcn1_hist.history.keys() if 'acc'  in s and 'val' not in s]
        mvcn1_val_acc  = [s for s in mvcn1_hist.history.keys() if 'acc'  in s and 'val' in s]
        
        ## As loss always exists
        epochs = range(1,self.epochs + 1)

        ## Training Loss
        self.plot_single('Training Loss', epochs, vox_hist, vcn1_hist, mvcn1_hist, \
                        vox_loss, vcn1_loss, mvcn1_loss)

        ## Validation Loss
        self.plot_single('Validation Loss', epochs, vox_hist, vcn1_hist, mvcn1_hist, \
                        vox_val_loss, vcn1_val_loss, mvcn1_val_loss)
        
        ## Training Accuracy
        self.plot_single('Training Accuracy', epochs, vox_hist, vcn1_hist, mvcn1_hist, \
                        vox_acc, vcn1_acc, mvcn1_acc)

        ## Validation Accuracy
        self.plot_single('Validation Accuracy', epochs, vox_hist, vcn1_hist, mvcn1_hist, \
                        vox_val_acc, vcn1_val_acc, mvcn1_val_acc)

    def plot_confusion_matrix(self, cm, classes, normalize=False, cmap=plt.cm.Blues):
        """
        This function prints and plots the confusion matrix.
        Normalization can be applied by setting `normalize=True`.
        """
        if normalize:
            cm = cm.astype('float') / cm.sum(axis=1)[:, np.newaxis]
            title='Normalized confusion matrix'
        else:
            title='Confusion matrix'

        self.plt_num += 1
        plt.figure(self.plt_num, figsize=(7.5, 6))
        plt.imshow(cm, interpolation='nearest', cmap=cmap)
        plt.title(title)
        plt.colorbar()
        tick_marks = np.arange(len(classes))
        plt.xticks(tick_marks, classes, rotation=45)
        plt.yticks(tick_marks, classes)

        fmt = '.2f' if normalize else 'd'
        thresh = cm.max() / 2.
        for i, j in itertools.product(range(cm.shape[0]), range(cm.shape[1])):
            plt.text(j, i, format(cm[i, j], fmt), horizontalalignment="center", color="white" if cm[i, j] > thresh else "black")

        plt.tight_layout()
        plt.ylabel('True label')
        plt.xlabel('Predicted label')

    # multiclass or binary report, If binary (sigmoid output), set binary parameter to True
    def full_multiclass_report(self, model, x, y_true, classes, batch_size=128, binary=False):
        # 1. Transform one-hot encoded y_true into their class number
        if not binary:
            y_true = np.argmax(y_true, axis=1)

        # 2. Predict classes and stores in y_pred
        # if self.model_net == 'V_CNN_2':
        #     y_pred = model.predict(x, batch_size=batch_size)
        #     y_pred = np.argmax(y_pred, axis=1)
        y_pred = model.predict_classes(x, batch_size=batch_size)
        
        # 3. Print accuracy score
        # accuracy = np.mean(y_pred==y_true)
        print("Accuracy : "+ str(accuracy_score(y_true, y_pred)))
        print("")

        # 4. Print classification report
        print("Classification Report")
        print(classification_report(y_true, y_pred, digits=3))

        # 5. Plot confusion matrix
        cnf_matrix = confusion_matrix(y_true, y_pred)
        print(cnf_matrix.shape)
        self.plot_confusion_matrix(cnf_matrix, classes=classes)

    def run(self):
        data, labels                                     = self.load_data(self.pcl_dataset)
        (X_train,    y_train),    (X_test, y_test)       = self.preprocess_data(data, labels)
        (X_train_2D, y_train_2D), (X_test_2D, y_test_2D) = self.preprocess_2D_data(data, labels)


        # preprocess
        self.number_class   = np.unique(y_test).shape[0]
        rospy.loginfo('Number of class : {}'.format(self.number_class))
        self.le             = LabelEncoder()
        classes             = np.array(['left_arm_top', 'right_arm_top'])
        self.encoded_labels = self.le.fit_transform(classes)

        # training
        model_voxnet   = self.voxnet((X_train, y_train), (X_test, y_test), batch_size=self.batch_size, epochs=self.epochs)
        voxnet_history = self.history

        model_vcn1     = self.V_CNN_1((X_train_2D, y_train_2D), (X_test_2D, y_test_2D), batch_size=self.batch_size, epochs=self.epochs) 
        vcnn1_history  = self.history
        
        model_mvcn1    = self.MV_CNN_1((X_train, y_train), (X_test, y_test), batch_size=self.batch_size, epochs=self.epochs) 
        mvcnn1_history = self.history

        self.plot_history(voxnet_history, vcnn1_history, mvcnn1_history)

        del model_voxnet
        del model_vcn1
        del model_mvcn1

        # confusion matrix
        with open(self.voxnet_model) as json_file:
            json_config = json_file.read()
            model       = model_from_json(json_config)
        model.load_weights(self.voxnet_weight)
        self.full_multiclass_report(model, X_test, y_test, self.le.inverse_transform(np.arange(self.number_class)), batch_size=self.batch_size)

        with open(self.vcnn1_model) as json_file:
            json_config = json_file.read()
            model       = model_from_json(json_config)
        model.load_weights(self.vcnn1_weight)
        self.full_multiclass_report(model, X_test_2D, y_test_2D, self.le.inverse_transform(np.arange(self.number_class)), batch_size=self.batch_size)

        with open(self.mvcnn1_model) as json_file:
            json_config = json_file.read()
            model       = model_from_json(json_config)
        model.load_weights(self.mvcnn1_weight)
        self.full_multiclass_report(model, X_test, y_test, self.le.inverse_transform(np.arange(self.number_class)), batch_size=self.batch_size)

        # show graph
        plt.show(block=False)
        input('Close: ')
        plt.close('all')

if __name__ == '__main__':
    dl = Deep_Learning()
    dl.run()