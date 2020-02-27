#!/usr/bin/env python3

import os
import sys
import cv2
import pickle
import rospy
import rospkg
import numpy as np
from cv2 import aruco
from time import sleep
from std_msgs.msg import Bool, Int16
from geometry_msgs.msg import Pose2D

class Arucos:
    def __init__(self, save):
        self.save        = self.str_to_bool(save)
        self.rospack     = rospkg.RosPack()

        self.data_path   = self.rospack.get_path("pioneer_main") + "/data/wolf_walk"
        self.save_data   = False
        self.top_frame   = 0

        # self.camera_type = 'C922'
        self.camera_type = 'C930E'

        self.load_calibrate()

        # video capture
        self.cap   = cv2.VideoCapture("/dev/{}".format(self.camera_type))
        self.cap.set(3, self.img_size[0]) # 1024 (frame width)
        self.cap.set(4, self.img_size[1]) # 576  (frame height)

        # update camera matrix
        self.update_matrix()

        # aruco dictionary
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        self.parameters = aruco.DetectorParameters_create()

        # parameter
        self.buffer_pts = 64
        self.pts        = []
        self.aruco_pose = Pose2D()

        self.init_save()
        
        ## Publisher
        self.top_frame_pub = rospy.Publisher('/pioneer/wolf/top_frame', Int16,   queue_size=10)
        self.aruco_pos_pub = rospy.Publisher('/pioneer/wolf/aruco_pos', Pose2D,  queue_size=10)

        ## Subscriber
        rospy.Subscriber('/pioneer/wolf/save_data', Bool, self.save_data_callback)

    def init_save(self):
        if self.save:
            sleep(2)
            n_folder  = len(os.walk(self.data_path).__next__()[1]) - 1
            data_path = "{}/{}".format(self.data_path, n_folder)
            cam_file  = "{}/wolf_top_cam-{}.avi" .format(data_path, n_folder)
            tra_file  = "{}/wolf_trajectory-{}.avi" .format(data_path, n_folder)
            fourcc    = cv2.VideoWriter_fourcc(*'MJPG')
            self.out  = cv2.VideoWriter(cam_file, fourcc, 30, (self.frame_width, self.frame_height))
            self.out1 = cv2.VideoWriter(tra_file, fourcc, 30, (self.frame_width, self.frame_height))
        else:
            return

    def str_to_bool(self, s):
        if s == 'true':
            return True
        elif s == 'false':
            return False
        else:
            raise ValueError # evil ValueError that doesn't tell you what the wrong value was

    def save_data_callback(self, msg):
        self.save_data = msg.data

    def load_calibrate(self):
        calib_path   = self.rospack.get_path("pioneer_vision") + "/data"
        camera_dist  = "{}/{}_dist.p".format(calib_path, self.camera_type)

        with open(camera_dist, "rb") as f:
            dist_pickle = pickle.load(f)
            print('Successful load: {}'.format(camera_dist))

        self.mtx      = dist_pickle["mtx"]
        self.dist     = dist_pickle["dist"]
        self.img_size = dist_pickle["img_size"]

    def update_matrix(self):
        _, img   = self.cap.read()
        img_gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
        h,  w    = img_gray.shape[:2]
        print('Original image size: {}, {}'.format(w, h) )

        self.new_mtx, self.roi = cv2.getOptimalNewCameraMatrix(self.mtx, self.dist, (w,h), 1, (w,h))
        self.frame_width  = self.roi[2]
        self.frame_height = self.roi[3]
        print('Cropped Image size: {}, {}'.format(self.frame_width, self.frame_height) )

    def undistort_image(self, img):
        undist_img =  cv2.undistort(img, self.mtx, self.dist, None, self.new_mtx)
            
        # crop undistorted field
        x, y, w, h = self.roi
        return undist_img[y:y+h, x:x+w]
    
    # Checks if a matrix is a valid rotation matrix.
    def is_rotation_matrix(self, R) :
        Rt = np.transpose(R)
        shouldBeIdentity = np.dot(Rt, R)
        I = np.identity(3, dtype = R.dtype)
        n = np.linalg.norm(I - shouldBeIdentity)
        return n < 1e-6
    
    # Calculates rotation matrix to euler angles
    def rotation_matrix_to_euler_angles(self, R) :
        assert(self.is_rotation_matrix(R))     
        sy = np.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])     
        singular = sy < 1e-6 
        if  not singular :
            x = np.arctan2(R[2,1] , R[2,2])
            y = np.arctan2(-R[2,0], sy)
            z = np.arctan2(R[1,0], R[0,0])
        else :
            x = np.arctan2(-R[1,2], R[1,1])
            y = np.arctan2(-R[2,0], sy)
            z = 0
    
        return np.array([x, y, z])

    def save_frame(self, frame, aruco_frame):
        if self.save_data:
            self.out.write(frame)
            self.out1.write(aruco_frame)

            self.top_frame += 1
            self.top_frame_pub.publish(self.top_frame)
            cv2.putText(aruco_frame, 'Rec.', (20, 40), cv2.FONT_HERSHEY_TRIPLEX, 0.8, (0, 0, 255), lineType=cv2.LINE_AA)

        return aruco_frame

    def aruco_process(self, frame):
        gray_img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray_img, self.aruco_dict, parameters=self.parameters)

        if np.all(ids != None):
            ids = np.squeeze(ids)
    
            # check if marker ids 0
            m_id = np.where(ids == 0)[0]

            # if found marker ids 0
            if m_id.size != 0:
                idx = m_id[0]
      
                M = cv2.moments(corners[idx])

                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                center = (cx, cy)

                rvec, tvec ,_ = aruco.estimatePoseSingleMarkers(corners[idx], 0.05, self.mtx, self.dist)
                aruco.drawAxis(frame, self.mtx, self.dist, rvec, tvec, 0.1)

                # draw center circle
                cv2.circle(frame, center, 5, self.orange, -1)

                # rotation matrix
                rmat, _    = cv2.Rodrigues(rvec)
                rx, ry, rz = np.degrees(self.rotation_matrix_to_euler_angles(rmat))

                # coordinate & angle text
                cv2.putText(frame, "x:{:.0f}"  .format(cx), (cx+40, cy),    cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.red, 1)
                cv2.putText(frame, "y:{:.0f}"  .format(cy), (cx+40, cy+20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.red, 1)
                cv2.putText(frame, "rz: {:.2f}".format(rz), (cx+40, cy+40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.red, 1)

                if self.save_data:
                    self.aruco_pose.x     = cx
                    self.aruco_pose.y     = cy
                    self.aruco_pose.theta = rz

                    self.aruco_pos_pub.publish(self.aruco_pose)
                    self.pts.append(center)

        # loop over the set of tracked points
        for i in np.arange(1, len(self.pts)):
            # if either of the tracked points are None, ignore
            if self.pts[i - 1] is None or self.pts[i] is None:
                continue
            
            thickness = 5 #int(np.sqrt(self.buffer_pts / float(i + 1)) * 2.5)
            cv2.line(frame, self.pts[i - 1], self.pts[i], self.orange, thickness)
            
        return frame

    def run(self):
        # color parameter
        self.orange = (0, 144, 255)
        self.red    = (0, 0, 255)

        while not rospy.is_shutdown():
            _, img    = self.cap.read()
            # frame    = img.copy()               # original frame
            frame     = self.undistort_image(img)  # calibrated frame
            ori_frame = frame.copy()

            # aruco process
            aruco_frame = self.aruco_process(frame)

            # save frame
            final_frame = self.save_frame(ori_frame, aruco_frame)

            # show frame
            cv2.imshow("aruco", final_frame)

            # wait key
            k = cv2.waitKey(1)
            if k == 27 or k == ord('q'):
                print('Exit with code 0')
                break

        cv2.destroyAllWindows()

if __name__ == '__main__':
    rospy.init_node('pioneer_top', anonymous=False)
    
    # if using ros launch length of sys.argv is 4
    if len(sys.argv) == 4:
        save = sys.argv[1]
        rospy.loginfo("[Top] Pioneer Wolf Top - Running")
        rospy.loginfo("[Top] Save Data : {}\n".format(save))

        arucos = Arucos(save)
        arucos.run() 
    else:
        rospy.logerr("[Top] Exit Argument not fulfilled")