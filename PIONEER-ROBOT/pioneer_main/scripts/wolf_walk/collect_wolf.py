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

        rospack              = rospkg.RosPack()
        self.pcl_path        = rospack.get_path("pioneer_main") + "/data/wolf_object/raw_pcl/"
        self.pcl_cam_path    = rospack.get_path("pioneer_main") + "/data/wolf_object/camera/"
        self.display_path    = rospack.get_path("pioneer_main") + "/data/wolf_object/display/"

        if not os.path.exists(self.pcl_path):
            os.mkdir(self.pcl_path)

        if not os.path.exists(self.pcl_cam_path):
            os.mkdir(self.pcl_cam_path)

        self.motion         = Motion()
        self.camera         = Camera()
        self.main_rate      = rospy.Rate(30)
        self.thread_rate    = rospy.Rate(30)

        self.scan_offset    = 50
        self.init_head_p    = 30 
        self.point_clouds   = None
        self.fourcc         = cv2.VideoWriter_fourcc(*'MJPG')
        self.frame_width    = self.camera.source_image.shape[0]
        self.frame_height   = self.camera.source_image.shape[1]
        self.thread1_flag   = False
        self.shutdown       = False
        self.visual_ptk     = None
        
        # labeling data
        self.objects        = [ 'small_suitcase', 'big_suitcase', 'black_chair', 'blue_chair', 'table', 'combo' ]
        self.target_obj     = None
        
        ## Publisher
        self.tts_pub        = rospy.Publisher('/robotis/sensor/text_to_speech', String, queue_size=10)

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
        cam_file = self.pcl_cam_path + self.target_obj + "-" + str(counter) + ".avi" 

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

    def play_sound(self, counter):
        sound      = String()
        sound.data = 'start {} {}'.format(self.target_obj, counter)
        self.tts_pub.publish(sound)

    def capture_data(self, cnt=-1):
        motion = self.motion
        try:
            self.visual_ptk.close()
        except:
            pass

        counter = len(os.walk(self.pcl_path).__next__()[2])
        self.play_sound(counter) # play start sound

        self.thread1_flag = False
        thread1 = threading.Thread(target = self.thread_record_frames, args =(lambda : self.thread1_flag, ))  
        thread1.start()
        sleep(2)

        motion.publisher_(motion.move_lidar_pub, "start") # scan full head_p
        sleep(1)
        motion.publisher_(motion.override_orig_pos_lidar_pub,  self.init_head_p) # overide original lidar pose
        self.wait_robot(motion, "Finish head joint in order to make pointcloud")

        # save lidar pcl
        pcl_file  = self.pcl_path + self.target_obj + "-" + str(counter) + ".npz"
        temp_name = self.target_obj + "-" + str(counter) + ".npz"
        self.plot_point_cloud(temp_name, self.point_clouds) # <-- plot
        np.savez(pcl_file, pcl=self.point_clouds, label=self.objects.index(self.target_obj))
        rospy.loginfo('[CW] pcl save: {}'.format(pcl_file))
        
        sleep(2)
        self.thread1_flag = True # kill record frames thread
        sleep(0.5)

    def capture_display_data(self, cnt=-1):
        motion = self.motion
        try:
            self.visual_ptk.close()
        except:
            pass

        motion.publisher_(motion.move_lidar_pub, "start") # scan full head_p
        sleep(1)
        motion.publisher_(motion.override_orig_pos_lidar_pub,  self.init_head_p) # overide original lidar pose
        self.wait_robot(motion, "Finish head joint in order to make pointcloud")

        # save lidar pcl
        pcl_file  = self.display_path + self.target_obj + ".npz"
        temp_name = self.target_obj + "-" + ".npz"
        self.plot_point_cloud(temp_name, self.point_clouds) # <-- plot
        np.savez(pcl_file, pcl=self.point_clouds, label=self.objects.index(self.target_obj))
        rospy.loginfo('[CW] pcl save: {}'.format(pcl_file))

        # capture image frame
        image_file  = self.display_path + self.target_obj + ".jpg"
        frame       = self.camera.source_image.copy()
        cv2.imwrite(image_file, frame)

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

                state = input("Proceed [auto or manual]: ")
                if state == 'q' or state == 'exit':
                    rospy.loginfo('[CW] Exit..')
                    self.stop_record = True # kill record frames thread
                    self.shutdown    = True
                    break

                elif state == 'auto':
                    name = input("select object {}: ".format(self.objects))
                    if name in self.objects:
                        self.target_obj = name
                    else:
                        rospy.logwarn('[CW] wrong arm input, Exit code')
                        break
                    rospy.loginfo('[CW] {}'.format(self.objects))

                    auto_counter = input("Total counter : ")
                    try:
                        auto_counter = int(auto_counter)
                        rospy.loginfo('[CW] Total counter: {}'.format(auto_counter))
                    except:
                        rospy.logwarn('[CW] wrong counter, Exit code')
                        break

                    sleep(5)

                    for cnt in range(auto_counter):
                        print()
                        cnt += 1
                        rospy.loginfo('[CW] Auto counter: {}'.format(cnt))
                        
                        # sleep(1)
                        self.capture_data(cnt)
                    
                elif state == 'manual':
                    # collect data manually
                    name = input("select object {}: ".format(self.objects))
                    if name in self.objects:
                        self.target_obj = name
                        self.capture_data()
                    else:
                        rospy.logwarn('[CW] wrong arm input, Exit code')

                else:
                    # collect data manually
                    name = input("select object {}: ".format(self.objects))
                    if name in self.objects:
                        self.target_obj = name
                        self.capture_display_data()
                    else:
                        rospy.logwarn('[CW] wrong arm input, Exit code')

                self.main_rate.sleep()

        except KeyboardInterrupt:
            self.stop_record = True # kill record frames thread
            self.shutdown    = True

if __name__ == '__main__':
    collect = Collect_Data()
    collect.run()