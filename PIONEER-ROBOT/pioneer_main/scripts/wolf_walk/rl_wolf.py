#!/usr/bin/env python3

import os
import sys
import math
import yaml
import rospy
import rospkg
import pickle

import random
import threading
import numpy as np
import matplotlib.pyplot as plt

from time import sleep
from geometry_msgs.msg import Pose2D
from std_msgs.msg import String, Bool, Int16

from pioneer_motion.action import Action
from pioneer_motion.motion import Motion
from pioneer_dragging.deepQlearn import DQN
from pioneer_dragging.environment import Env
from pioneer_utils.excel_rl_wolf import Excel

class Wolf_Walk:
    def __init__(self, save):
        self.rospack     = rospkg.RosPack()

        # save file
        self.save_data   = self.str_to_bool(save)
        self.save_config_init()

        self.main_rate   = rospy.Rate(60)
        self.action      = Action("Thormang3_Wolf")
        self.motion      = Motion()
        
        # Reinforncement Learning
        self.reinforcement_init()

        # parameter
        # self.surface = '1.Plywood'
        self.surface = '2.Carpet'
        # self.surface = '3.Tile'
        self.object = ['default', 'office_chair', 'office_chair_heavy', 'foot_chair', 'small_suitcase', 's_heavy_suitcase', 'big_suitcase', 'human_suitcase']
       
        # self.pull_motion = 'default'
        # self.pull_motion = 'office_chair'
        # self.pull_motion = 'office_chair_heavy'
        # self.pull_motion = 'foot_chair'
        # self.pull_motion = 'small_suitcase'
        # self.pull_motion = 's_heavy_suitcase'
        # self.pull_motion = 'big_suitcase'
        self.pull_motion = 'human_suitcase'
        
        self.debug       = False
        self.state       = None
        self.walk_mode   = False
        self.com_action  = []
        self.start_robot = True
        self.start_pose  = Pose2D()
        self.finish_pose = Pose2D()
        self.thread_rate = rospy.Rate(30)

        # calibration path
        self.calib_path  = self.rospack.get_path("pioneer_main") + "/config/wolf_trajectory.yaml"

        # figure
        self.figure_init()

        ## Publisher
        self.save_pub    = rospy.Publisher('/pioneer/wolf/save_data', Bool,  queue_size=10)

        ## Subscriber
        rospy.Subscriber('/pioneer/wolf/state',         String,  self.state_callback)
        rospy.Subscriber('/pioneer/wolf/robot_frame',   Int16,   self.robot_frame_callback)
        rospy.Subscriber('/pioneer/wolf/tripod_frame',  Int16,   self.tripod_frame_callback)
        rospy.Subscriber('/pioneer/wolf/top_frame',     Int16,   self.top_frame_callback)
        rospy.Subscriber('/pioneer/wolf/aruco_pos',     Pose2D,  self.aruco_pos_callback)

    def str_to_bool(self, s):
        if s == 'true':
            return True
        elif s == 'false':
            return False
        else:
            raise ValueError # evil ValueError that doesn't tell you what the wrong value was

    def save_config_init(self): 
        if self.save_data:
            data_path         = self.rospack.get_path("pioneer_main") + "/data/wolf_walk"
            self.n_folder     = len(os.walk(data_path).__next__()[1])
            self.data_path    = "{}/{}".format(data_path, self.n_folder)

            self.robot_frame  = 0
            self.tripod_frame = 0
            self.top_frame    = 0
            self.aruco_pose   = Pose2D()
            self.aruco_pose.x = 0
            self.aruco_pose.y = 0
            self.aruco_pose.theta = 0

            self.rsense_cnt   = 0
            self.thread1_flag = False
            
            if not os.path.exists(self.data_path):
                os.mkdir(self.data_path)

            self.excel        = Excel('{}/wolf_data_{}.xlsx'.format(self.data_path, self.n_folder))
            self.figure1      = "{}/wolf_com_x-{}.png"      .format(self.data_path, self.n_folder)
            self.figure2      = "{}/wolf_trajectory-{}.png" .format(self.data_path, self.n_folder)
            self.figure1_data = "{}/wolf_com_x-{}.p"        .format(self.data_path, self.n_folder)
            self.figure2_data = "{}/wolf_trajectory-{}.p"   .format(self.data_path, self.n_folder)

            # self.lidar_file = "{}/wolf_lidar_pcl-{}.npz"     .format(self.data_path, self.n_folder)
            # self.yaml_file  = "{}/wolf_initial_pose-{}.yaml" .format(self.data_path, self.n_folder)

            rospy.loginfo('[WW] Data path: {}'.format(self.data_path))
        else:
            return

    def reinforcement_init(self):
        self.ft_sensor   = rospy.get_param("/ft_sensor")
        self.env         = Env(self.ft_sensor)
        self.n_states    = self.env.observation_space
        self.n_actions   = self.env.action_space.n
        
        # create Deep Q-Network
        self.dqn         = DQN(self.n_states, self.n_actions)
        
        # load weight
        rl_path          = self.rospack.get_path("pioneer_dragging") + "/data"
        username         = rospy.get_param("/username")
        n_folder         = rospy.get_param("/n_folder") 
        self.dqn.file_models = "{0}/{1}-{2}/{2}-pytorch-RL.tar".format(rl_path, username, n_folder)
        self.dqn.load_model()

    def figure_init(self):
        # self.style_plot  = random.choice(plt.style.available)
        self.style_plot  = 'bmh'
        plt.style.use(self.style_plot)
        plt.ion()

        self.red   = 'tab:red'
        self.green = 'tab:green'
        self.blue  = 'tab:blue'
        self.black = 'k'

        title_1 = "CoM X (Obj:{}, F/T Sens:{})".format(self.pull_motion, self.ft_sensor)
        self.fig1, self.ax1 = self.create_figure(figure_n=1, title=title_1, x_label='Step', y_label='Offset Value')

        title_2 = "Trajectory (Obj:{}, F/T Sens:{})".format(self.pull_motion, self.ft_sensor)
        self.fig2, self.ax2 = self.create_figure(figure_n=2, title=title_2, x_label='x', y_label='y')
        self.ax2.set_xlim([0, 1024])
        self.ax2.set_ylim([0, 576])

    def create_figure(self, figure_n, title, x_label, y_label):
        fig = plt.figure(figure_n)
        ax  = fig.add_subplot(1,1,1)
        ax.set_title (title)
        ax.set_xlabel(x_label)
        ax.set_ylabel(y_label)
        return fig, ax
    
    def state_callback(self, msg):
        self.state = msg.data
        rospy.loginfo("[WW] Received: {}".format(self.state))
    
    def robot_frame_callback(self, msg):
        self.robot_frame = msg.data

    def tripod_frame_callback(self, msg):
        self.tripod_frame = msg.data

    def top_frame_callback(self, msg):
        self.top_frame = msg.data

    def aruco_pos_callback(self, msg):
        self.aruco_pose = msg

        if self.start_robot:
            self.start_pose  = msg
            self.start_robot = False

        self.finish_pose = msg

    def wait_robot(self, obj, msg):
        while obj.status_msg != msg:
            pass # do nothing

    def wait_action(self):
        sleep(0.5)
        while not self.action.finish_action:
            pass

    def thread_logging_data(self, stop_thread):
        rate_cnt = 0

        while not rospy.is_shutdown():
            rate_cnt += 1
            sensor   = self.env.sensor
            walking  = self.env.walking

            self.excel.add_data(no = rate_cnt, \
                                imu_roll    = sensor.imu_ori['roll'],   imu_pitch    = sensor.imu_ori['pitch'],  imu_yaw    = sensor.imu_ori['yaw'],    \
                                lf_x        = sensor.left_torque['x'],  lf_y         = sensor.left_torque['y'],  lf_z       = sensor.left_torque['z'],  \
                                rf_x        = sensor.right_torque['x'], rf_y         = sensor.right_torque['y'], rf_z       = sensor.right_torque['z'], \
                                des_roll    = walking.des_pose_roll,    des_pitch    = walking.des_pose_pitch,   cob_x      = self.env.cob_x,           \
                                pose_x      = self.aruco_pose.x,        pose_y       = self.aruco_pose.y,        pose_theta = self.aruco_pose.theta,    \
                                top_frame   = self.top_frame,                                                                                           \
                                robot_frame = self.robot_frame,         tripod_frame = self.tripod_frame,        rs_frame = self.rsense_cnt)
        
            if stop_thread():
                rospy.loginfo("[Env] Thread log data killed")
                break
            self.thread_rate.sleep()

    def calculate_trajectory(self):
        self.ax2.plot(self.start_pose.x,    self.start_pose.y,  'o', color=self.red)
        self.ax2.plot(self.finish_pose.x,   self.finish_pose.y, 'o', color=self.blue)
        self.ax2.plot([self.start_pose.x,   self.finish_pose.x], [self.start_pose.y, self.finish_pose.y], '-', color=self.black)

        self.ax2.text(self.start_pose.x,    self.start_pose.y-50,  "X:{}, Y:{}".format(self.start_pose.x, self.start_pose.y))
        self.ax2.text(self.finish_pose.x,   self.finish_pose.y-50, "X:{}, Y:{}".format(self.finish_pose.x, self.finish_pose.y))

        # convert pixel to meter
        with open(self.calib_path, 'r') as f:
            pixel_config = yaml.safe_load(f)
            rospy.loginfo('[ww] Loaded pixel config: {}'.format(self.calib_path))

        xs_pixel = pixel_config['calib_x']['x_start']
        xe_pixel = pixel_config['calib_x']['x_end']
        x_meter  = pixel_config['calib_x']['x_meter']
        x_pixel_to_m = x_meter / abs(xe_pixel - xs_pixel) # x pixel to meter

        ys_pixel = pixel_config['calib_y']['y_start']
        ye_pixel = pixel_config['calib_y']['y_end']
        y_meter  = pixel_config['calib_y']['y_meter']
        y_pixel_to_m = y_meter / abs(ye_pixel - ys_pixel) # y pixel to meter

        x_dist_m = abs(self.finish_pose.x - self.start_pose.x) * x_pixel_to_m
        y_dist_m = abs(self.finish_pose.y - self.start_pose.y) * y_pixel_to_m

        # euclidean distance in meter
        final_dist_m = math.sqrt( math.pow(x_dist_m, 2) + math.pow(y_dist_m, 2) )
        rospy.loginfo("[WW] Robot finish walk: {:.4f} meter".format(final_dist_m))

        mid_x    = (self.finish_pose.x + self.start_pose.x) / 2
        mid_y    = (self.finish_pose.y + self.start_pose.y) / 2
        euc_dist = np.linalg.norm( np.array([self.finish_pose.x, self.finish_pose.y]) - np.array([self.start_pose.x, self.start_pose.y]) )

        self.ax2.text(mid_x, mid_y-50, "Dist Pixel: {:.4f}".format(euc_dist))
        self.ax2.text(mid_x, mid_y-75, "Dist M: {:.4f}".    format(final_dist_m))
        plt.draw()
        plt.pause(0.01)

        self.tra_dict = { 'x_start'    : self.start_pose.x,   'y_start' : self.start_pose.y,
                          'x_finish'   : self.finish_pose.x,  'y_finish': self.finish_pose.y,
                          'distance_m' : final_dist_m}

    def run(self):
        motion  = self.motion
        action  = self.action
        env     = self.env
        dqn     = self.dqn

        # self.state = 'ini_pose'
        if self.pull_motion == 'default':
            self.state = 'walk_mode'
        else:
            self.state = 'pull_pose'
        sleep(2)

        while not rospy.is_shutdown():

            # first state
            if self.state == 'ini_pose':
                rospy.loginfo('[WW] Robot State : {}'.format(self.state))
                self.walk_mode = False

                motion.publisher_(motion.init_pose_pub, "ini_pose", latch=True)
                self.wait_robot(motion, "Finish Init Pose")

                if self.debug:
                    self.state = None
                else:
                    sleep(2)
                    if self.pull_motion == 'default':
                        self.state = 'walk_mode'
                    else:
                        self.state = 'pull_pose'
                    
            # second state
            elif self.state == 'pull_pose':
                rospy.loginfo('[WW] Robot State : {}'.format(self.state))
                self.walk_mode = False
                
                action.motor.publisher_(action.motor.module_control_pub, "none", latch=True)
                # action.set_init_config(torque=50)
                action.play_motion(self.pull_motion)
                self.wait_action()

                if self.debug:
                    self.state = None
                else:
                    self.state = 'walk_mode'

            # third state
            elif self.state == 'walk_mode':
                rospy.loginfo('[WW] Robot State : {}'.format(self.state))
                
                # set walk mode, and turn on balance
                env.initial()
                self.walk_mode = True
                
                if self.debug:
                    self.state = None
                else:
                    self.state = 'walk_backward'

            # forth state
            elif self.state == 'walk_backward':
                rospy.loginfo('[WW] Robot State : {}'.format(self.state))

                if self.walk_mode:
                    if self.save_data:
                        self.save_pub.publish(True)  # turn on record robot, tripod, top
                        # thread logging data
                        thread1 = threading.Thread(target = self.thread_logging_data, args =(lambda : self.thread1_flag, )) 
                        thread1.start()
                    state = env.reset()

                    while True:
                        action_rl = dqn.test_action(state)
                        next_state, done, cob_x = env.step(action_rl)
                        rospy.loginfo('[WW] action: {}'.format(action_rl))
                        self.com_action.append(cob_x)
                        self.ax1.plot(self.com_action, 'o-', color=self.green)
                        plt.draw()
                        plt.pause(0.01)
                        if not done:
                            state = next_state
                        else:
                            break
                    rospy.loginfo('[WW] Robot finish walk')

                    if self.debug:
                        self.state = None
                    else:
                        self.state = 'release'
                else:
                    rospy.logwarn('[WW] Set walk model first !')
                    self.state = None

            # last state
            elif self.state == 'release':
                rospy.loginfo('[WW] Robot State : {}'.format(self.state))
                self.walk_mode      = False

                # calculate trajectory
                self.calculate_trajectory()
                
                if self.save_data:
                    pickle.dump( self.com_action, open( self.figure1_data, "wb" ) )
                    pickle.dump( self.tra_dict,   open( self.figure2_data, "wb" ) )
                    
                    self.fig1.savefig(self.figure1, dpi=self.fig1.dpi)
                    self.fig2.savefig(self.figure2, dpi=self.fig2.dpi)
                    rospy.loginfo('[RL] Save figure1: {}'.format(self.figure1))
                    rospy.loginfo('[RL] Save figure2: {}'.format(self.figure2))

                    self.save_pub.publish(False)  # turn off record robot & tripod
                    self.thread1_flag = False

                # set to default CoM
                self.env.update_COM(rospy.get_param('/cob_x'))
                sleep(2)
                self.env.update_COM(rospy.get_param('/cob_x'))

                if self.pull_motion != 'default' and self.pull_motion != 'office_chair' \
                    and self.pull_motion != 'office_chair_heavy' :
                    action.motor.publisher_(action.motor.module_control_pub, "none", latch=True)
                    action.play_motion(self.state)
                    self.wait_action()
                    rospy.loginfo('[WW] Release motion')

                self.state = 'finish'
            
            elif self.state == 'finish':
                rospy.loginfo('[WW] Robot State : {}'.format(self.state))
                rospy.loginfo('[WW] Finish')

                motion.publisher_(motion.init_pose_pub, "ini_pose", latch=True)
                self.wait_robot(motion, "Finish Init Pose")

                # copy to backup directory
                target_path  = "/media/ahan/Data/4.Research/4.Thormang3/3.Data/7.Wolf_Walk/100_data"
                surface_path = "{}/{}".format(target_path, self.surface)
                if not os.path.exists(surface_path):
                    os.mkdir(surface_path)

                index = self.object.index(self.pull_motion)
                object_path  = "{}/{}.{}".format(surface_path, index, self.pull_motion)
                if not os.path.exists(object_path):
                    os.mkdir(object_path)

                if self.ft_sensor:
                    final_path  = "{}/with_FT/".format(object_path)
                else:
                    final_path  = "{}/without_FT/".format(object_path)
                if not os.path.exists(final_path):
                    os.mkdir(final_path)

                cmd = "cp -r {} {}".format(self.data_path, final_path)
                os.system(cmd)
                print(cmd)

                self.state = None

            self.main_rate.sleep()
        
if __name__ == '__main__':
    rospy.init_node('pioneer_wolf_walk')

    # if using ros launch length of sys.argv is 4
    if len(sys.argv) == 4:
        save = sys.argv[1]
        rospy.loginfo("[Wolf] Pioneer Main Wolf Walk - Running")
        rospy.loginfo("[WW] Save Data : {}\n".format(save))

        wolf = Wolf_Walk(save)
        wolf.run()
    else:
        rospy.logerr("[WW] Exit Argument not fulfilled")