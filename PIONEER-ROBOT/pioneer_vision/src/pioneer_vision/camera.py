#!/usr/bin/env python

import cv2
import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class Camera:
    def __init__(self):
        # rospy.init_node('pioneer_camera', anonymous=False)
        self.bridge = CvBridge()
        self.source_image = np.zeros((rospy.get_param("/uvc_camera_center_node/height"), rospy.get_param("/uvc_camera_center_node/width"), 3), np.uint8)
        # self.result_image = None
             
        ## Subscriber
        rospy.Subscriber('/robotis/sensor/camera/image_raw', Image, self.images_callback)

        ## Publisher
        self.pioneer_img_pub = rospy.Publisher("/pioneer_vision/image", Image, queue_size=1)

    def images_callback(self, img):
        try:
            self.source_image = self.bridge.imgmsg_to_cv2(img, "bgr8")
        except CvBridgeError as e:
            rospy.loginfo(e)
 
    def run(self):
        rate = rospy.Rate(60)
        while not rospy.is_shutdown():
            # self.result_image = self.source_image.copy()
            # print(self.source_image.shape)
            self.pioneer_img_pub.publish( self.bridge.cv2_to_imgmsg(self.source_image, "bgr8") )
            rate.sleep()       

if __name__ == '__main__':
    c = Camera()
    c.run()
