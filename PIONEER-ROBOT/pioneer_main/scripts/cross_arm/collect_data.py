#!/usr/bin/env python3

import os
import cv2
import pptk
import threading
import rospy
import rospkg
import ros_numpy
import numpy as np
from time import sleep
from std_msgs.msg import String, Bool
from sensor_msgs.msg import PointCloud2
from pioneer_vision.camera import Camera
from pioneer_motion.motion import Motion
from pioneer_motion.action import Action

class Collect_Data:
    def __init__(self):
        rospy.init_node('pioneer_cross_collect_data', disable_signals=True)
        rospy.loginfo("[CA] Pioneer Collect Data Cross Arm- Running")

        rospack           = rospkg.RosPack()
        self.pcl_path     = rospack.get_path("pioneer_main") + "/data/cross_arm/raw_pcl/"
        self.pcl_cam_path = rospack.get_path("pioneer_main") + "/data/cross_arm/cam/"
        
        self.motion       = Motion()
        self.action       = Action()
        self.camera       = Camera()
        self.main_rate    = rospy.Rate(30)
        self.thread_rate  = rospy.Rate(30)

        self.scan_offset  = 50
        self.init_head_p  = 0 
        self.point_clouds = None
        self.fourcc       = cv2.VideoWriter_fourcc(*'MJPG')
        self.frame_width  = self.camera.source_image.shape[0]
        self.frame_height = self.camera.source_image.shape[1]
        self.thread1_flag = False

        # labeling data
        self.arm = 'left_arm_top'
        # self.arm = 'right_arm_top'

        self.shutdown   = False
        self.visual_ptk = None

        ## Subscriber
        rospy.Subscriber('/robotis/sensor/assembled_scan', PointCloud2, self.point_cloud2_callback)

    def point_cloud2_callback(self, data):
        pc          = ros_numpy.numpify(data)
        points      = np.zeros((pc.shape[0],4))
        points[:,0] = pc['x']
        points[:,1] = pc['y']
        points[:,2] = pc['z']
        points[:,3] = pc['intensities']
        # print(pc.dtype.names) # ('x', 'y', 'z', 'intensities', 'index')
        # print(len(points))
        self.point_clouds = points

    def wait_robot(self, obj, msg):
        while obj.status_msg != msg:
            if self.shutdown:
                break
            else:
                pass # do nothing

    def thread_record_frames(self, stop_thread):
        counter  = len(os.walk(self.pcl_cam_path).__next__()[2])
        cam_file = self.pcl_cam_path + self.arm + "-" + str(counter) + ".avi" 
        rospy.loginfo('[Data] start save: {}'.format(cam_file))
        out = cv2.VideoWriter(cam_file, self.fourcc, 30, (self.frame_width, self.frame_height))

        while not stop_thread():
            frame = self.camera.source_image.copy()
            out.write(frame)
            # cv2.waitKey(50)
            self.thread_rate.sleep()

        rospy.loginfo('[Data] finish save: {}'.format(cam_file))
    
    def plot_point_cloud(self, label, pcl_data, big_point=False, color=True):
        rospy.loginfo("[Data] {} - length pcl : {}".format(label, pcl_data.shape))
        self.visual_ptk = pptk.viewer(pcl_data[:,:3])
        
        if color:
            self.visual_ptk.attributes(pcl_data[:,-1])
        if big_point:
            self.visual_ptk.set(point_size=0.0025)

    def wait_action(self):
        while not self.action.finish_action:
            pass
            
    def run(self):
        motion = self.motion
        action = self.action

        action.motor.publisher_(action.motor.module_control_pub, "none", latch=True)
        action.play_motion("standby")
        self.wait_action()
        rospy.loginfo('[Data] Finish head init')

        action.set_velocity(0)
        motion.publisher_(motion.module_control_pub, "head_control_module", latch=True)
        sleep(1)

        try:
            while not rospy.is_shutdown():
                print()

                state = input("Proceed : ")
                if state == 'q':
                    rospy.loginfo('[Data] Exit..')
                    self.stop_record = True # kill record frames thread
                    self.shutdown    = True
                    break

                else:
                    try:
                        self.visual_ptk.close()
                    except:
                        pass
                    
                    self.thread1_flag = False
                    thread1 = threading.Thread(target = self.thread_record_frames, args =(lambda : self.thread1_flag, ))  
                    thread1.start()
                    sleep(2)

                    motion.publisher_(motion.move_lidar_pub, "start") # scan full head_p
                    sleep(0.5)
                    # motion.publisher_(motion.override_original_pos_lidar_pub,  self.init_head_p) # overide original lidar pose
                    # motion.publisher_(motion.move_lidar_range_pub, np.radians(self.scan_offset+1)) # scan head with range
                    self.wait_robot(motion, "Finish head joint in order to make pointcloud")

                    counter   = len(os.walk(self.pcl_path).__next__()[2])
                    pcl_file  = self.pcl_path + self.arm + "-" + str(counter) + ".npz"
                    temp_name = self.arm + "-" + str(counter) + ".npz"
                    self.plot_point_cloud(temp_name, self.point_clouds) # <-- plot

                    np.savez(pcl_file, pcl=self.point_clouds)
                    rospy.loginfo('[Data] pcl save: {}'.format(pcl_file))

                    sleep(2)
                    self.thread1_flag = True # kill record frames thread
                    sleep(0.5)

                self.main_rate.sleep()

        except KeyboardInterrupt:
            self.stop_record = True # kill record frames thread
            self.shutdown    = True

if __name__ == '__main__':
    collect = Collect_Data()
    collect.run()