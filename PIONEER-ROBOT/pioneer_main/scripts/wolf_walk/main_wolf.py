#!/usr/bin/env python3

import os
import cv2
import yaml
import time
import pptk
import rospy
import rospkg
import threading
import numpy as np
from time import sleep
from std_msgs.msg import String, Bool, Int16
from pioneer_motion.action import Action
from pioneer_vision.camera import Camera
from pioneer_motion.motion import Motion
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
            self.excel      = Excel('{}/wolf_data_{}.xlsx'   .format(self.data_path, self.n_folder))
            self.cam_file   = "{}/wolf_walk_cam-{}.avi"      .format(self.data_path, self.n_folder)
            self.lidar_file = "{}/wolf_lidar_pcl-{}.npz"     .format(self.data_path, self.n_folder)
            self.yaml_file  = "{}/wolf_initial_pose-{}.yaml" .format(self.data_path, self.n_folder)

            rospy.loginfo('[WW] Data path: {}'.format(self.data_path))

        self.main_rate    = rospy.Rate(10)
        self.sensor       = Sensor("Thormang3_Wolf")
        self.action       = Action("Thormang3_Wolf")
        self.motion       = Motion()
        self.camera       = Camera()
        self.walking      = Walking()

        self.mutex        = threading.Lock()
        self.thread_rate  = rospy.Rate(15)
        self.thread1_flag = False
        self.scan_finish  = False
        self.visual_ptk1  = None
        self.robot_frame  = 0
        self.tripod_frame = 0

        ## Publisher
        self.save_pub  = rospy.Publisher('/pioneer/wolf/save_data', Bool,  queue_size=10)

        ## Subscriber
        rospy.Subscriber('/robotis/sensor/move_lidar',  String,  self.lidar_turn_callback)
        rospy.Subscriber('/pioneer/wolf/tripod_frame',  Int16,   self.tripod_frame_callback)

    def wait_robot(self, obj, msg):
        while obj.status_msg != msg:
            pass # do nothing

    def lidar_turn_callback(self, msg):
        if msg.data == "end":
            rospy.loginfo("[WW] Lidar finish scan")

            while self.sensor.lidar_pcl is None:
                pass
            self.plot_point_cloud('wolf_lidar_pcl', self.sensor.lidar_pcl, hardware='lidar') # <-- plot

            if self.save_data:
                np.savez(self.lidar_file, pcl=self.sensor.lidar_pcl)
                rospy.loginfo('[WW] Saved lidar pcl data: {}'.format(self.lidar_file))

    def tripod_frame_callback(self, msg):
        self.tripod_frame = msg.data
    
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
            self.robot_frame += 1
            frame  = self.camera.source_image.copy()
            out.write(frame)
            self.thread_rate.sleep()
            self.mutex.release()

    def set_initial_pose(self):
        r_foot_x     = rospy.get_param("/initial_pose/right_foot/x")
        r_foot_y     = rospy.get_param("/initial_pose/right_foot/y")
        r_foot_z     = rospy.get_param("/initial_pose/right_foot/z")
        r_foot_roll  = rospy.get_param("/initial_pose/right_foot/roll")
        r_foot_pitch = rospy.get_param("/initial_pose/right_foot/pitch")
        r_foot_yaw   = rospy.get_param("/initial_pose/right_foot/yaw")

        l_foot_x     = rospy.get_param("/initial_pose/left_foot/x")
        l_foot_y     = rospy.get_param("/initial_pose/left_foot/y")
        l_foot_z     = rospy.get_param("/initial_pose/left_foot/z")
        l_foot_roll  = rospy.get_param("/initial_pose/left_foot/roll")
        l_foot_pitch = rospy.get_param("/initial_pose/left_foot/pitch")
        l_foot_yaw   = rospy.get_param("/initial_pose/left_foot/yaw")

        cob_x        = rospy.get_param("/initial_pose/centre_of_body/x")
        cob_y        = rospy.get_param("/initial_pose/centre_of_body/y")
        cob_z        = rospy.get_param("/initial_pose/centre_of_body/z")
        cob_roll     = rospy.get_param("/initial_pose/centre_of_body/roll")
        cob_pitch    = rospy.get_param("/initial_pose/centre_of_body/pitch")
        cob_yaw      = rospy.get_param("/initial_pose/centre_of_body/yaw")

        self.walking.set_robot_pose( r_foot_x, r_foot_y, r_foot_z, r_foot_roll, r_foot_pitch, r_foot_yaw,\
                                     l_foot_x, l_foot_y, l_foot_z, l_foot_roll, l_foot_pitch, l_foot_yaw,\
                                     cob_x,    cob_y,    cob_z,    cob_roll,    cob_pitch,    cob_yaw)
        rospy.loginfo('[WW] Finish set initial pose')

        if self.save_data:
            right_foot     = { 'x': r_foot_x, 'y': r_foot_y, 'z': r_foot_z, 'roll': r_foot_roll, 'pitch': r_foot_pitch, 'yaw': r_foot_yaw }
            left_foot      = { 'x': l_foot_x, 'y': l_foot_y, 'z': l_foot_z, 'roll': l_foot_roll, 'pitch': l_foot_pitch, 'yaw': l_foot_yaw }
            centre_of_body = { 'x': cob_x,    'y': cob_y,    'z': cob_z,    'roll': cob_roll,    'pitch': cob_pitch,    'yaw': cob_yaw    }

            initial_config = {}
            initial_config['right_foot']     = left_foot
            initial_config['left_foot']      = right_foot
            initial_config['centre_of_body'] = centre_of_body
            
            with open(self.yaml_file, 'w') as f:
                yaml.dump(initial_config, f, default_flow_style=False)

    def wait_action(self):
        sleep(0.5)
        while not self.action.finish_action:
            pass

    def run(self):
        sensor = self.sensor
        motion = self.motion
        action = self.action
        walk   = self.walking

        motion.publisher_(motion.init_pose_pub, "ini_pose", latch=True)
        self.wait_robot(motion, "Finish Init Pose")

        input("Press enter to scan lidar: ") # wait user
        motion.publisher_(motion.module_control_pub, "head_control_module", latch=True)
        motion.publisher_(motion.move_lidar_pub, "start") # scan full head_p
        self.wait_robot(motion, "Finish head joint in order to make pointcloud")
        sleep(1)

        input("Press enter to grap object: ") # wait user
        action.motor.publisher_(action.motor.module_control_pub, "none", latch=True)
        action.set_init_config(torque=50)
        sleep(1)
        action.play_motion("han")

        input("Press enter for walk mode: ") # wait user
        # walk.publisher_(walk.walking_pub, "ini_pose", latch=True)
        # self.wait_robot(walk, "Finish Init Pose")

        walk.publisher_(walk.walking_pub, "set_mode")
        self.wait_robot(walk, "Walking_Module_is_enabled")
        walk.publisher_(walk.walking_pub, "balance_on")
        self.wait_robot(walk, "Balance_Param_Setting_Finished")
        self.wait_robot(walk, "Joint_FeedBack_Gain_Update_Finished")

        # set robot walking initial pose
        self.set_initial_pose()
        sleep(2)
        walk.publisher_(walk.walking_pub, "balance_on")
        self.wait_robot(walk, "Balance_Param_Setting_Finished")
        self.wait_robot(walk, "Joint_FeedBack_Gain_Update_Finished")

        input("Press enter for start walking: ") # wait user
        walk.walk_command("forward", 2, 1.0, 0.1, 0.05, 5)

        if self.save_data:
            cnt, curr, last = 0, 0, 0
            excel   = self.excel
            thread1 = threading.Thread(target = self.thread_record_frames, args =(lambda : self.thread1_flag, ))  
            thread1.start()

            self.save_pub.publish(True)  # record tripod camera

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
                                        robot_frame = self.robot_frame,    tripod_frame = self.tripod_frame)

                        # saved pcl
                        np.savez("{}/wolf_realsense_pcl-{}-{}.npz".format(self.data_path, self.n_folder, cnt), pcl=sensor.real_sense_pcl)
                        rospy.loginfo('[WW] Saving data: {}'.format(cnt))
                    last = curr

                except:
                    pass
                  
            self.main_rate.sleep()

        self.thread1_flag = True
        
if __name__ == '__main__':
    wolf = Wolf_Walk()
    wolf.run()