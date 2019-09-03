#!/usr/bin/env python3

import cv2
import yaml
import rospy
import rospkg
import numpy as np
from cv2 import aruco
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point32

class Aruco:
    def __init__(self):
        rospy.init_node('pioneer_aruco', anonymous=False)

        self.rospack        = rospkg.RosPack()
        self.aruco_ws_path  = self.rospack.get_path("pioneer_main") + "/config/thormang3_aruco_ws.yaml"
        self.source_img     = np.zeros((rospy.get_param("/uvc_camera_center_node/width"), rospy.get_param("/uvc_camera_center_node/height"), 3), np.uint8)
        self.main_rate      = rospy.Rate(20)
        self.res_img        = None
        self.l_point        = None
        self.r_point        = None
        self.left_ws        = None
        self.right_ws        = None

        ## Publisher
        self.left_aruco_pos_pub  = rospy.Publisher("/pioneer/aruco/left_position",   Point32, queue_size=1)
        self.right_aruco_pos_pub = rospy.Publisher("/pioneer/aruco/right_position",  Point32, queue_size=1)
        self.left_arm_pos_pub    = rospy.Publisher("/pioneer/aruco/left_arm_point",  Point32, queue_size=1)
        self.right_arm_pos_pub   = rospy.Publisher("/pioneer/aruco/right_arm_point", Point32, queue_size=1)

        ## Subscriber
        rospy.Subscriber('/robotis/sensor/camera/image_raw', Image, self.images_callback)

    def images_callback(self, img):
        dt  = np.dtype(np.uint8)
        dt  = dt.newbyteorder('>')
        arr = np.frombuffer(img.data,dtype=dt)
        arr = np.reshape(arr,(480,640,3))
        self.source_img = cv2.cvtColor(arr, cv2.COLOR_BGR2RGB)

    def target_point(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.check_roi('left_arm', (x, y), self.left_ws)

        if event == cv2.EVENT_RBUTTONDOWN:
            self.check_roi('right_arm', (x, y), self.right_ws)

    def check_roi(self, arm, points, area):
        area = area.reshape((4,2))

        if arm == 'left_arm':
            x_min_b  = area[0][0]
            x_min_a  = area[1][0]
            x_max_a  = area[2][0]
            x_max_b  = area[3][0]
        elif arm == 'right_arm':
            x_max_b  = area[0][0]
            x_max_a  = area[1][0]
            x_min_a  = area[2][0]
            x_min_b  = area[3][0]

        y_frame_min = area[1][1]
        y_frame_max = area[0][1]

        x_frame_min = np.interp( points[1], [y_frame_min, y_frame_max], [x_min_a, x_min_b] )
        x_frame_min = int( np.round(x_frame_min, 0) )
        x_frame_max = np.interp( points[1], [y_frame_min, y_frame_max], [x_max_a, x_max_b] )
        x_frame_max = int( np.round(x_frame_max, 0) )

        # Y Check
        if points[1] >= y_frame_min and points[1] <= y_frame_max:
            # X Check
            if points[0] >= x_frame_min and points[0] <= x_frame_max:
                tar_pos   = Point32()
                tar_pos.x = points[0]
                tar_pos.y = points[1]

                if arm == 'left_arm':
                    self.l_point = (points[0], points[1])
                    self.left_arm_pos_pub.publish(tar_pos)
                elif arm == 'right_arm':
                    self.r_point = (points[0], points[1])
                    self.right_arm_pos_pub.publish(tar_pos)
        #     else:
        #         rospy.logerr("[{0}] X Out of range".format(arm))
        # else:
        #     rospy.logerr("[{0}] Y Out of range".format(arm))

    def load_config(self, arm):
        try:
            with open(self.aruco_ws_path, 'r') as f:
                aruco_ws = yaml.safe_load(f)
        except yaml.YAMLError as exc:
            print(exc)
        try:
            config = aruco_ws[arm]
            pts    = []
            for i in ['P1', 'P2', 'P3', 'P4']:
                pts.append( [config[i]['cx'], config[i]['cy']] )
            pts = np.array( pts, np.int32)

            y_frame_min = np.mean([ pts[1][1], pts[2][1] ], dtype=int)
            y_frame_max = np.mean([ pts[0][1], pts[3][1] ], dtype=int)
            pts[1][1] = pts[2][1] = y_frame_min
            pts[0][1] = pts[3][1] = y_frame_max

            # rospy.loginfo('[Aruco] Load {0} workspace successfully'.format(arm))
            return pts.reshape((-1,1,2))
        except:
            return None

    def run(self):
        rospy.loginfo("[Aruco] Pioneer Calibration Aruco")
        aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
        parameters = aruco.DetectorParameters_create()
        left_aruco_pos  = Point32()
        right_aruco_pos = Point32()

        cv2.namedWindow("image")
        cv2.setMouseCallback("image", self.target_point)

        # load polygon
        self.left_ws  = self.load_config('left_arm')
        self.right_ws = self.load_config('right_arm')
        
        while not rospy.is_shutdown():
            src_img     = self.source_img.copy()
            gray_img    = cv2.cvtColor(src_img, cv2.COLOR_BGR2GRAY)
            corners, ids, rejectedImgPoints = aruco.detectMarkers(gray_img, aruco_dict, parameters=parameters)
            
            left_aruco_pos.x  = left_aruco_pos.y  = -1
            right_aruco_pos.x = right_aruco_pos.y = -1

            if len(corners) > 0:
                if ids[0,0] == 0:
                    M = cv2.moments(corners[0])
                    left_aruco_pos.x = int(M["m10"] / M["m00"])
                    left_aruco_pos.y = int(M["m01"] / M["m00"])
                elif ids[0,0] == 1:
                    M = cv2.moments(corners[0])
                    right_aruco_pos.x = int(M["m10"] / M["m00"])
                    right_aruco_pos.y = int(M["m01"] / M["m00"])
               
                if len(corners) > 1:
                    if ids[1,0] == 0:
                        M = cv2.moments(corners[1])
                        left_aruco_pos.x = int(M["m10"] / M["m00"])
                        left_aruco_pos.y = int(M["m01"] / M["m00"])
                    elif ids[1,0] == 1:
                        M = cv2.moments(corners[1])
                        right_aruco_pos.x = int(M["m10"] / M["m00"])
                        right_aruco_pos.y = int(M["m01"] / M["m00"])

            self.left_aruco_pos_pub.publish(left_aruco_pos)
            self.right_aruco_pos_pub.publish(right_aruco_pos)

            self.res_img = aruco.drawDetectedMarkers(src_img.copy(), corners, ids)
            cv2.polylines(self.res_img,[self.left_ws], True, (255,0,0), 2)
            cv2.polylines(self.res_img,[self.right_ws], True, (0,0,255), 2)

            cv2.circle(self.res_img, self.l_point, 5, (255,0,0), -1) # Left arm point
            cv2.circle(self.res_img, self.r_point, 5, (0,0,255), -1) # Right arm point
            cv2.imshow('image', self.res_img)

            self.main_rate.sleep()
            k = cv2.waitKey(1)
            if k == ord('q'):
                break
            if k == ord('l'):
                rospy.loginfo("[Aruco] Load Workspace Map")
                self.left_ws  = self.load_config('left_arm')
                self.right_ws = self.load_config('right_arm')
                
        rospy.loginfo("[Aruco] Shutdown")

if __name__ == '__main__':
    aruco_ws = Aruco()
    aruco_ws.run()