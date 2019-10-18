#!/usr/bin/env python3

import cv2
import os
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
from pioneer_kinematics.kinematics import Kinematics

class Collect_Data:
    def __init__(self):
        rospy.init_node('pioneer_cross_arm', disable_signals=True)
        rospy.loginfo("[CA] Pioneer Collect Data Cross Arm- Running")

        rospack           = rospkg.RosPack()
        self.pcl_path     = rospack.get_path("pioneer_main") + "/data/cross_arm/raw_pcl/"
        self.pcl_cam_path = rospack.get_path("pioneer_main") + "/data/cross_arm/cam/"
        
        self.kinematics   = Kinematics()
        self.motion       = Motion()
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

        self.shutdown = False

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
        rospy.loginfo('[CAL] start save: {}'.format(cam_file))
        out = cv2.VideoWriter(cam_file, self.fourcc, 30, (self.frame_width, self.frame_height))

        while not stop_thread():
            frame = self.camera.source_image.copy()
            out.write(frame)
            # cv2.waitKey(50)
            self.thread_rate.sleep()

        rospy.loginfo('[CAL] finish save: {}'.format(cam_file))

    def run(self):
        kinematics = self.kinematics
        motion     = self.motion

        kinematics.publisher_(kinematics.module_control_pub, "manipulation_module", latch=True)  # <-- Enable Manipulation mode
        rospy.loginfo('[CA] Manipulation start init pose')
        kinematics.publisher_(kinematics.send_ini_pose_msg_pub, "ini_pose", latch=True)
        self.wait_robot(kinematics, "End Init Trajectory")
        rospy.loginfo('[CA] Manipulation finish init pose')

        sleep(1)

        motion.publisher_(motion.module_control_pub, "head_control_module", latch=True)
        motion.set_head_joint_states(['head_y', 'head_p'], [0, self.init_head_p])
        self.wait_robot(motion, "Head movement is finished.")
        rospy.loginfo('[CA] Finish head init')

        self.first_run = True

        try:
            while not rospy.is_shutdown():
                print()
                state = input("Proceed : (Press Enter)")
                if state == 'q':
                    break

                else:
                    self.thread1_flag = False
                    thread1 = threading.Thread(target = self.thread_record_frames, args =(lambda : self.thread1_flag, ))  
                    thread1.start()
                    sleep(1)

                    motion.set_head_joint_states(['head_y', 'head_p'], [0, 85])
                    self.wait_robot(motion, "Head movement is finished.")

                    if self.first_run:
                        sleep(1.5)
                        self.first_run = False
                    
                    motion.publisher_(motion.move_lidar_pub, "start") # scan full head_p
                    sleep(1)
                    motion.publisher_(motion.override_original_pos_lidar_pub,  self.init_head_p) # overide original lidar pose
                    # motion.publisher_(motion.move_lidar_range_pub, np.radians(self.scan_offset+1)) # scan head with range
                    self.wait_robot(motion, "Finish head joint in order to make pointcloud")

                    counter  = len(os.walk(self.pcl_path).__next__()[2])
                    pcl_file = self.pcl_path + self.arm + "-" + str(counter) + ".npz" 
                    np.savez(pcl_file, pcl=self.point_clouds)
                    rospy.loginfo('[CAL] pcl save: {}'.format(pcl_file))

                    sleep(1)
                    self.thread1_flag = True # kill record frames thread
                    sleep(0.5)

                self.main_rate.sleep()

        except KeyboardInterrupt:
            self.stop_record = True # kill record frames thread
            self.shutdown    = True

if __name__ == '__main__':
    collect = Collect_Data()
    collect.run()