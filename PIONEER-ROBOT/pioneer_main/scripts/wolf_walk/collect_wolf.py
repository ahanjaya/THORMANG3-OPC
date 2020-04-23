#!/usr/bin/env python3

import os
import cv2
import pptk
import threading
import rospy
import rospkg
import ros_numpy
import numpy as np
from gtts import gTTS
from time import sleep
from std_msgs.msg import String, Bool
from sensor_msgs.msg import PointCloud2
from pioneer_vision.camera import Camera
from pioneer_motion.motion import Motion

class Collect_Data:
    def __init__(self):
        rospy.init_node('pioneer_wolf_collect_data', disable_signals=True)
        rospy.loginfo("[CW] Pioneer Wolf Collect Lidar Data - Running")

        rospack           = rospkg.RosPack()
        self.pcl_path     = rospack.get_path("pioneer_main") + "/data/wolf_object/raw_pcl"
        self.cam_path     = rospack.get_path("pioneer_main") + "/data/wolf_object/videos"
        self.img_path     = rospack.get_path("pioneer_main") + "/data/wolf_object/images"

        self.init_folder(self.pcl_path)
        self.init_folder(self.cam_path)
        self.init_folder(self.img_path)

        self.motion       = Motion()
        self.camera       = Camera()
        self.main_rate    = rospy.Rate(30)
        self.thread_rate  = rospy.Rate(30)

        self.scan_offset  = 50
        self.init_head_p  = 30 
        self.point_clouds = None
        self.fourcc       = cv2.VideoWriter_fourcc(*'MJPG')
        self.frame_width  = self.camera.source_image.shape[0]
        self.frame_height = self.camera.source_image.shape[1]
        self.thread1_flag = False
        self.record_video = False
        self.shutdown     = False
        self.visual_ptk   = None
        
        # labeling data
        self.objects      = [ 'small_suitcase', 'big_suitcase', 'black_chair', 'blue_chair', 'table', 'combo' ]
        self.target_obj   = None
        
        ## Publisher
        self.tts_pub      = rospy.Publisher('/robotis/sensor/text_to_speech', String, queue_size=10)

        ## Subscriber
        rospy.Subscriber('/robotis/sensor/assembled_scan', PointCloud2, self.point_cloud2_callback)

    def init_folder(self, path):
        if not os.path.exists(path):
            os.mkdir(path)
            rospy.loginfo('[CW] Created dir: {}'.format(path))

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
        counter  = len(os.walk(self.cam_path).__next__()[2])
        cam_file = self.cam_path + self.target_obj + "-" + str(counter) + ".avi" 

        rospy.loginfo('[CW] start save: {}'.format(cam_file))
        out = cv2.VideoWriter(cam_file, self.fourcc, 30, (self.frame_width, self.frame_height))

        while not stop_thread():
            frame = self.camera.source_image.copy()
            out.write(frame)
            # cv2.waitKey(50)
            self.thread_rate.sleep()

        rospy.loginfo('[CW] finish save: {}'.format(cam_file))
    
    def plot_point_cloud(self, label, pcl_data, big_point=False, color=True):
        rospy.loginfo("[CW] {} - length pcl : {}".format(label, pcl_data.shape))
        self.visual_ptk = pptk.viewer(pcl_data[:,:3])
        
        if color:
            self.visual_ptk.attributes(pcl_data[:,-1])
        if big_point:
            self.visual_ptk.set(point_size=0.0025)
        
        # use self.get_pptk_attributes for detail
        self.visual_ptk.set(r=2.88)
        self.visual_ptk.set(phi=3.24)
        self.visual_ptk.set(theta=0.64)
        self.visual_ptk.set(lookat=(0.78, -0.01, -0.05))
        self.visual_ptk.set(show_axis=False)

    def play_sound(self, counter):
        sound      = String()
        sound.data = '{} {}'.format(self.target_obj, counter)
        self.tts_pub.publish(sound)

    def capture_data(self):
        motion = self.motion
        self.close_plot()

        counter = len(os.walk(self.pcl_path).__next__()[2])

        # publish sound to PPC
        self.play_sound(counter) # play start sound

        if self.record_video:
            self.thread1_flag = False
            thread1 = threading.Thread(target = self.thread_record_frames, args =(lambda : self.thread1_flag, ))  
            thread1.start()
            sleep(2)

        # start scan
        motion.publisher_(motion.move_lidar_pub, "start") # scan full head_p
        sleep(1)
        motion.publisher_(motion.override_orig_pos_lidar_pub,  self.init_head_p) # overide original lidar pose
        self.wait_robot(motion, "Finish head joint in order to make pointcloud")
        
        # plot
        pcl_file    = self.target_obj + "-" + str(counter) + ".npz"
        self.plot_point_cloud(pcl_file, self.point_clouds) # <-- plot

        # save lidar pcl
        pcl_file    = "{}/{}-{}.jpg".format(self.pcl_path, self.target_obj, counter)
        np.savez(pcl_file, pcl=self.point_clouds, label=self.objects.index(self.target_obj))
        rospy.loginfo('[CW] pcl save: {}'.format(pcl_file))

        # save last image frame
        image_file  = "{}/{}-{}.jpg".format(self.img_path, self.target_obj, counter)
        frame       = self.camera.source_image.copy()
        cv2.imwrite(image_file, frame)

        if self.record_video:
            sleep(2)
            self.thread1_flag = True 

        sleep(0.5)

    def get_pptk_attributes(self, visual_ptk):
        attributes = ['lookat', 'phi', 'r', 'theta']

        for i in attributes:
            temp = visual_ptk.get(i)
            print('{}: {}'.format(i, temp))

    def close_plot(self):
        if self.visual_ptk is not None:
            self.visual_ptk.close()

    def run(self):
        motion = self.motion

        motion.publisher_(motion.init_pose_pub, "ini_pose", latch=True)
        self.wait_robot(motion, "Finish Init Pose")
        rospy.loginfo('[CW] finish init pose')

        motion.publisher_(motion.module_control_pub, "head_control_module", latch=True)
        sleep(2)

        try:
            while not rospy.is_shutdown():
                print()

                state = input("Proceed [auto, manual, or exit]: ")
                if state == 'q' or state == 'exit':
                    rospy.loginfo('[CW] Exit..')
                    self.stop_record = True # kill record frames thread
                    self.shutdown    = True
                    self.close_plot()
                    break

                elif state == 'auto':
                    # name = input("select object {}: ".format(self.objects))
                    name = 'black_chair'
                    if name in self.objects:
                        self.target_obj = name
                        rospy.loginfo('[CW] object: {}'.format(name))
                    else:
                        rospy.logwarn('[CW] wrong object name, please retry')
                        continue

                    auto_counter = input("Total counter : ")
                    try:
                        auto_counter = int(auto_counter)
                        rospy.loginfo('[CW] Total counter: {}'.format(auto_counter))
                    except:
                        rospy.logwarn('[CW] wrong counter, please retry')
                        continue

                    # preparation to start capture data
                    sleep(5)

                    # loop capturing data
                    for counter in range(auto_counter):
                        print()
                        rospy.loginfo('[CW] Auto counter: {}'.format(counter))
                        self.capture_data()
                        sleep(3)
                    
                elif state == 'manual':
                    name = input("select object {}: ".format(self.objects))
                    # name = 'big_suitcase'
                    if name in self.objects:
                        self.target_obj = name
                        self.capture_data()
                    else:
                        rospy.logwarn('[CW] wrong object name, Exit code')

                elif state == 'info':
                    if self.visual_ptk is not None:
                        self.get_pptk_attributes(self.visual_ptk)

                else:
                    rospy.logwarn('[CW] invalid command, please retry')

                self.main_rate.sleep()

        except KeyboardInterrupt:
            self.stop_record = True # kill record frames thread
            self.shutdown    = True

if __name__ == '__main__':
    collect = Collect_Data()
    collect.run()