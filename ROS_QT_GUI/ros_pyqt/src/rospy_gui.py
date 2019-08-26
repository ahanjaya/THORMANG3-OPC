#!/usr/bin/env python

import sys
import rospy
from std_msgs.msg import String
from PyQt4 import QtGui
from PyQt4.QtGui import QLabel, QVBoxLayout, QHBoxLayout, QSlider, QPushButton
from PyQt4.QtCore import Qt

class PyGui(QtGui.QWidget):
    def __init__(self):
        super(PyGui, self).__init__()
        self.setObjectName('PyGui')
        # self.pub = rospy.Publisher("pyqt_topic", String, queue_size=10)
        self.init_pose_pub_ = rospy.Publisher("/robotis/base/ini_pose", String, queue_size=10)

        rospy.init_node('pyqt_gui')
        self.current_value = 0

        # Create Horizontal Layout
        my_hlay = QHBoxLayout()
        # Create Vertical Layout
        my_vlay = QVBoxLayout()

        # Init Button
        init_btn = QPushButton()
        init_btn.setText("Init Pose")
        init_btn.setFixedWidth(130)
        init_btn.clicked.connect(self.publish_init_topic)  
        # Add Horizontal Layout
        my_hlay.addWidget(init_btn)
        my_hlay.addSpacing(50)

        # self.my_label = QLabel()
        # self.my_label.setFixedWidth(140)
        # self.my_label.setText("num: " + str(0))
        # self.my_label.setEnabled(False)
        # # Add Horizontal Layout
        # my_hlay.addWidget(self.my_label)

        # # Slider
        # my_slider = QSlider()
        # my_slider.setMinimum(0)
        # my_slider.setMaximum(200)
        # my_slider.setOrientation(Qt.Horizontal)
        # my_slider.valueChanged.connect(self.changeValue)
        # # Add Vertical Layout
        # my_vlay.addWidget(my_slider)

        # Torque Off Button
        reset_btn = QPushButton()
        reset_btn.setText("Torque OFF")
        reset_btn.setFixedWidth(130)
        # reset_btn.clicked.connect(self.publish_topic)  
        # Add Horizontal Layout
        my_hlay.addWidget(reset_btn)


        # Display Layout
        layout = QVBoxLayout()
        layout.addLayout(my_hlay)
        layout.addLayout(my_vlay)
        
        self.setLayout(layout)
        # self.show()

    def publish_init_topic(self):
        # self.pub.publish(str(self.current_value))
        init_msg = "ini_pose"
        self.init_pose_pub_.publish(init_msg)

    def changeValue(self, value):
        self.my_label.setText("num: " + str(value))
        self.current_value = value


if __name__ == "__main__":
    app=QtGui.QApplication(sys.argv)
    pyShow = PyGui()
    pyShow.show()
    sys.exit(app.exec_())
