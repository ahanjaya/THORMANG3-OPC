#!/usr/bin/env python3

import os
import sys
import rospy
import rospkg
import pickle

import random
import threading
import numpy as np
import matplotlib.pyplot as plt

from time import sleep
from std_msgs.msg import String, Bool, Int16
from pioneer_motion.action import Action
from pioneer_motion.motion import Motion
from pioneer_dragging.deepQlearn import DQN
from pioneer_dragging.environment import Env
from pioneer_utils.export_excel_wolf import Excel

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
        self.pull_motion = 'big_suitcase'
        self.debug       = False
        self.state       = None
        self.walk_mode   = False
        self.com_action  = []
        self.thread_rate = rospy.Rate(30)

        # figure
        self.figure_init()

        ## Publisher
        self.save_pub    = rospy.Publisher('/pioneer/wolf/save_data', Bool,  queue_size=10)

        ## Subscriber
        rospy.Subscriber('/pioneer/wolf/state',         String,  self.state_callback)
        rospy.Subscriber('/pioneer/wolf/tripod_frame',  Int16,   self.tripod_frame_callback)

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

            self.tripod_frame = 0
            self.robot_frame  = 0
            self.rsense_cnt   = 0
            self.thread1_flag = False
            
            if not os.path.exists(self.data_path):
                os.mkdir(self.data_path)

            self.excel        = Excel('{}/wolf_data_{}.xlsx' .format(self.data_path, self.n_folder))
            self.figure1      = "{}/wolf_com_x-{}.png"       .format(self.data_path, self.n_folder)
            self.figure1_data = "{}/wolf_pickle_com_x-{}.p"  .format(self.data_path, self.n_folder)

            # self.lidar_file = "{}/wolf_lidar_pcl-{}.npz"     .format(self.data_path, self.n_folder)
            # self.yaml_file  = "{}/wolf_initial_pose-{}.yaml" .format(self.data_path, self.n_folder)

            rospy.loginfo('[WW] Data path: {}'.format(self.data_path))
        else:
            return

    def reinforcement_init(self):
        self.env         = Env()
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
        self.style_plot  = random.choice(plt.style.available)
        plt.style.use(self.style_plot)
        plt.ion()
        self.green = 'tab:green'

        self.fig1, self.ax1 = self.create_figure(figure_n=1, title="CoM X ({})".format(self.pull_motion),\
                                                x_label='Step', y_label='Offset Value')

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
    
    def tripod_frame_callback(self, msg):
        self.tripod_frame = msg.data

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
                                imu_roll    = sensor.imu_ori['roll'],   imu_pitch    = sensor.imu_ori['pitch'],  imu_yaw  = sensor.imu_ori['yaw'],    \
                                lf_x        = sensor.left_torque['x'],  lf_y         = sensor.left_torque['y'],  lf_z     = sensor.left_torque['z'],  \
                                rf_x        = sensor.right_torque['x'], rf_y         = sensor.right_torque['y'], rf_z     = sensor.right_torque['z'], \
                                des_roll    = walking.des_pose_roll,    des_pitch    = walking.des_pose_pitch,   cob_x    = self.env.cob_x,           \
                                robot_frame = self.robot_frame,         tripod_frame = self.tripod_frame,        rs_frame = self.rsense_cnt)
        
            if stop_thread():
                rospy.loginfo("[Env] Thread log data killed")
                break
            self.thread_rate.sleep()

    def run(self):
        motion  = self.motion
        action  = self.action
        env     = self.env
        dqn     = self.dqn

        self.state = 'ini_pose'
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
                        self.save_pub.publish(True)  # turn on record robot & tripod

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
                
                if self.save_data:
                    pickle.dump( self.com_action, open( self.figure1_data, "wb" ) )
                    
                    self.fig1.savefig(self.figure1, dpi=self.fig1.dpi)
                    rospy.loginfo('[RL] Save figure1: {}'.format(self.figure1))

                    self.save_pub.publish(False)  # turn off record robot & tripod
                    self.thread1_flag = False

                action.motor.publisher_(action.motor.module_control_pub, "none", latch=True)
                action.play_motion(self.state)
                self.wait_action()

                rospy.loginfo('[WW] Finish')
                self.state = None
                # break

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