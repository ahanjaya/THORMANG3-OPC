#!/usr/bin/env python3

import cv2
import os
import time
import pptk
import rospy
import rospkg
import threading
import numpy as np
from time import sleep
from pioneer_vision.camera import Camera
from pioneer_sensors.sensor import Sensor
from pioneer_walking.walking import Walking
from pioneer_utils.export_excel_wolf import Excel

class Wolf_Walk:
    def __init__(self):
        rospy.init_node('pioneer_wolf_walk') #, disable_signals=True)
        rospy.loginfo("[Wolf] Pioneer Main Wolf Walk - Running")

        rospack           = rospkg.RosPack()
        data_path         = rospack.get_path("pioneer_main") + "/data/wolf_walk"
        self.n_folder     = len(os.walk(data_path).__next__()[1])
        self.data_path    = "{}/{}".format(data_path, self.n_folder)
        
        self.save_data    = True
        if self.save_data:
            if not os.path.exists(self.data_path):
                os.mkdir(self.data_path)
            self.excel    = Excel('{}/wolf_data_{}.xlsx'.format(self.data_path, self.n_folder))
            rospy.loginfo('[WW] Data path: {}'.format(self.data_path))

        self.main_rate    = rospy.Rate(10)
        self.sensor       = Sensor("Thormang3_Wolf")
        self.camera       = Camera()
        self.walking      = Walking()
        self.cam_file     = "{}/wolf_walk_cam-{}.avi". format(self.data_path, self.n_folder)

        self.mutex        = threading.Lock()
        self.thread_rate  = rospy.Rate(15)
        self.thread1_flag = False
        self.scan_finish  = False
        self.visual_ptk1  = None
        self.frames_cnt   = 0


        ## Publisher

        ## Subscriber

    def wait_robot(self, obj, msg):
        while obj.status_msg != msg:
            pass # do nothing

    def plot_point_cloud(self, label, pcl_data, hardware):
        rospy.loginfo("[Wolf] {} - length pcl : {}".format(label, pcl_data.shape))
        visual_ptk = pptk.viewer(pcl_data[:,:3])

        if hardware == "lidar":
            visual_ptk.attributes(pcl_data[:,-1])
            visual_ptk.set(point_size=0.0025)
        elif hardware == "realsense":
            visual_ptk.set(point_size=0.0025)
            color = pcl_data[:, 3:]
            visual_ptk.attributes(color / 255)
        return visual_ptk

    def close_all(self):
        if self.visual_ptk1 is not None:
            self.visual_ptk1.close()

    def thread_record_frames(self, stop_thread):
        frame        = self.camera.source_image.copy()
        frame_height = frame.shape[0]
        frame_width  = frame.shape[1]
        fourcc       = cv2.VideoWriter_fourcc(*'MJPG')
        out          = cv2.VideoWriter(self.cam_file, fourcc, 30, (frame_width, frame_height))

        # if video is not saving, check frame shape of height and width
        # it might be swipe magicly

        while not stop_thread():
            self.mutex.acquire()
            self.frames_cnt += 1
            frame  = self.camera.source_image.copy()
            out.write(frame)
            self.thread_rate.sleep()
            self.mutex.release()

    def run(self):
        sensor = self.sensor
        walk   = self.walking

        walk.publisher_(walk.walking_pub, "ini_pose", latch=True)
        self.wait_robot(walk, "Finish Init Pose")
        walk.publisher_(walk.walking_pub, "set_mode")
        self.wait_robot(walk, "Walking_Module_is_enabled")
        walk.publisher_(walk.walking_pub, "balance_on")
        self.wait_robot(walk, "Balance_Param_Setting_Finished")
        self.wait_robot(walk, "Joint_FeedBack_Gain_Update_Finished")

        input("Proceed : ") # wait user

        if self.save_data:
            cnt, curr, last = 0, 0, 0
            excel   = self.excel
            thread1 = threading.Thread(target = self.thread_record_frames, args =(lambda : self.thread1_flag, ))  
            thread1.start()

        while not rospy.is_shutdown():
            if self.save_data:
                try :
                    curr = sensor.real_sense_pcl.shape[0]
                    if curr != last :
                        cnt += 1

                        # saved excel
                        excel.add_data(no = cnt, \
                                        imu_roll = sensor.imu_ori['roll'], imu_pitch = sensor.imu_ori['pitch'], imu_yaw = sensor.imu_ori['yaw'], \
                                        lf_x = sensor.left_torque['x'],    lf_y = sensor.left_torque['y'],      lf_z = sensor.left_torque['z'],  \
                                        rf_x = sensor.right_torque['x'],   rf_y = sensor.right_torque['y'],     rf_z = sensor.right_torque['z'], \
                                        frames_cnt = self.frames_cnt )

                        # saved pcl
                        np.savez("{}/wolf_pcl-{}-{}.npz".format(self.data_path, self.n_folder, cnt), pcl=sensor.real_sense_pcl)
                        rospy.loginfo('[WW] Saved data: {}'.format(cnt))
                    last = curr

                except:
                    pass
                  
            self.main_rate.sleep()

        self.thread1_flag = True
        
if __name__ == '__main__':
    wolf = Wolf_Walk()
    wolf.run()