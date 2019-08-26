#!/usr/bin/env python

import cv2
import rospy
import threading
import numpy as np
from sensor_msgs.msg import Image

class Camera:
    def __init__(self):
        rospy.init_node('pioneer_camera', anonymous=False)
        self.source_image = np.zeros((rospy.get_param("/uvc_camera_center_node/width"), rospy.get_param("/uvc_camera_center_node/height"), 3), np.uint8)
        self.thread_rate    = rospy.Rate(0.1)
        self.thread1_flag   = False
                     
        ## Publisher
        self.pioneer_img_pub = rospy.Publisher("/pioneer_vision/image", Image, queue_size=1)
        self.read_frames()

    def images_callback(self, img):
        dt = np.dtype(np.uint8)
        dt = dt.newbyteorder('>')
        arr = np.frombuffer(img.data,dtype=dt)
        arr = np.reshape(arr,(480,640,3))
        self.source_image = cv2.cvtColor(arr, cv2.COLOR_BGR2RGB)
    
    def kill_threads(self):
        self.thread1_flag = True
        
    def thread_read_frames(self, stop_thread):
         while True:
            ## Subscriber
            rospy.Subscriber('/robotis/sensor/camera/image_raw', Image, self.images_callback)
            self.thread_rate.sleep()
            self.pioneer_img_pub.publish( self.bridge.cv2_to_imgmsg(self.source_image, "bgr8") )
            if stop_thread():
                rospy.loginfo("[Camera] Thread killed")
                break

    def read_frames(self):
        thread1 = threading.Thread(target = self.thread_read_frames, args =(lambda : self.thread1_flag, ))  
        thread1.start()