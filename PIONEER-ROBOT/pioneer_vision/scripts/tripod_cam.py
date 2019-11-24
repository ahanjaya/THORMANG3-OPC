#!/usr/bin/env python3

import os
import cv2
import rospy
import rospkg
import numpy as np
from std_msgs.msg import Bool, Int16

class Tripod:
    def __init__(self):
        rospy.init_node('pioneer_tripod', anonymous=False)
        rospy.loginfo("[Tripod] Pioneer Wolf Tripod - Running")

        rospack           = rospkg.RosPack()
        self.data_path    = rospack.get_path("pioneer_main") + "/data/wolf_walk"
        self.save_data    = False
        self.frame_width  = 0
        self.frame_height = 0
        self.tripod_frame = 0

        ## Publisher
        self.tripod_frame_pub = rospy.Publisher('/pioneer/wolf/tripod_frame', Int16,  queue_size=10)

        ## Subscriber
        rospy.Subscriber('/pioneer/wolf/save_data', Bool, self.save_data_callback)

    def save_data_callback(self, msg):
        if msg.data == True:
            n_folder  = len(os.walk(self.data_path).__next__()[1]) - 1
            data_path = "{}/{}".format(self.data_path, n_folder)
            
            cam_file  = "{}/wolf_tripod_cam-{}.avi" .format(data_path, n_folder)
            fourcc    = cv2.VideoWriter_fourcc(*'MJPG')
            self.out  = cv2.VideoWriter(cam_file, fourcc, 30, (self.frame_width, self.frame_height))

            self.save_data = True
        
    def run(self):
        cap = cv2.VideoCapture(0)
        cap.set(3, 800)
        cap.set(4, 600)

        self.frame_width  = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

        while not rospy.is_shutdown():
            _, frame = cap.read()

            if self.save_data:
                self.out.write(frame)
                self.tripod_frame += 1
                self.tripod_frame_pub.publish(self.tripod_frame)

                cv2.putText(frame, 'Rec.', (20, 40), cv2.FONT_HERSHEY_TRIPLEX, 0.8, (0, 0, 255), lineType=cv2.LINE_AA)

            cv2.imshow('tripod frame', frame)
            ch = 0xFF & cv2.waitKey(1)
            if ch == 27:
                break

        cap.release()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    tripod = Tripod()
    tripod.run()