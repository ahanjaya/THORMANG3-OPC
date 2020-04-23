#! /usr/bin/env python3

import rospy
import threading
import numpy as np
from time import sleep
from gym import spaces, logger

from pioneer_sensors.sensor import Sensor
from pioneer_walking.walking import Walking

class Env:
    def __init__(self, ft_sensor):
        self.ft_sensor     = ft_sensor
        rospy.loginfo('[Env] F/T Sensor: {}'.format(self.ft_sensor))

        self.walking       = Walking()
        self.sensor        = Sensor("Thormang3_Wolf")

        self.fall_angle    = rospy.get_param('/fall_angle')
        self.cob_x         = -0.02 #-0.01 # -0.08 rospy.get_param('/cob_x')
        # -0.015 plwood
        # -0.01  carpet
        # -0.02  tile
        # -0.08  carpet human 

        self.step_size     = 0.03 #0.025 # 0.02 # rospy.get_param('/step_size')
        # 0.015 carpet
        # 0.02  plywood
        # 0.03  tile

        self.step_num      = 12 # rospy.get_param("/step_num") 
        self.thread_rate   = rospy.Rate(30)
        self.action_space  = spaces.Discrete(3)

        if self.ft_sensor:
            self.observation_space = 4
        else:
            self.observation_space = 2

    def wait_robot(self, obj, msg, msg1=None):
        if msg1 is None:
            while obj.status_msg != msg:
                if rospy.is_shutdown():
                    break
                else:
                    pass # do nothing
        else:
            while True:
                if obj.status_msg == msg or obj.status_msg == msg1:
                    break
                elif rospy.is_shutdown():
                    break
                else:
                    pass # do nothing

    def initial(self):
        sleep(1)

        walking = self.walking
        rospy.loginfo('[Env] Init Pose')

        # set walking mode
        walking.publisher_(walking.walking_pub, "set_mode")
        self.wait_robot(walking, "Walking_Module_is_enabled")
        rospy.loginfo('[Env] Set walk mode')

        # turn on balance
        self.update_COM(self.cob_x)
        self.wait_robot(self.walking, "Balance_Param_Setting_Finished")

        # walking.publisher_(walking.walking_pub, "balance_on")
        # self.wait_robot(self.walking, "Balance_Param_Setting_Finished", "Joint_FeedBack_Gain_Update_Finished")
        rospy.loginfo('[Env] Balance on')

        return

    def reset(self):
        walking = self.walking
        self.walk_finished = False
        # self.cob_x  = rospy.get_param('/cob_x')

        # start walking
        walking.walk_command("backward", self.step_num, 1.0, 0.1, 0.05, 5)
        self.wait_robot(walking, "Walking_Started")

        # start thread check walk
        thread2 = threading.Thread(target = self.thread_check_walk) 
        thread2.start()

        state = self.get_state()
        return state
    
    def thread_check_walk(self):
        while not rospy.is_shutdown():
            if self.walking.status_msg == "Walking_Finished":
                self.walk_finished = True
                rospy.loginfo('[Env] Walk finished')
                break
        
            self.thread_rate.sleep()

    def get_state(self):
        sensor = self.sensor

        # Force/Torque Sensor
        left_foot  = np.array([ sensor.left_torque['x'],  sensor.left_torque['y'],  sensor.left_torque['z']  ])
        right_foot = np.array([ sensor.right_torque['x'], sensor.right_torque['y'], sensor.right_torque['z'] ])
        origin     = np.array([0, 0, 0])

        # Binary state foot 
        # euclidean distance from torque vector
        euc_lfoot  = np.linalg.norm(left_foot - origin)
        euc_rfoot  = np.linalg.norm(right_foot - origin)

        if euc_lfoot <= 2:   
            state_lfoot = 0  # air
        else:
            state_lfoot = 1  # ground
        
        if euc_rfoot <= 6:
            state_rfoot = 0  # air
        else:
            state_rfoot = 1  # ground
        
        # rospy.loginfo('[Env] Left foot: {} \t Right foot: {}'.format(state_lfoot, state_rfoot) )

        # IMU 
        # imu_pitch = sensor.imu_ori['pitch']
        # imu_roll  = sensor.imu_ori['roll']

        imu_pitch = sensor.imu_ori['pitch']# + 1.8 #.0# 1.8 # 1.66
        
        if sensor.imu_ori['roll'] >= 0: 
            imu_roll  = 180 - sensor.imu_ori['roll']        # positive value fall to right
        else:
            imu_roll  = -1 * (180 + sensor.imu_ori['roll']) # negative value fall to left

        if self.ft_sensor:
            state = np.array([ imu_pitch, imu_roll, state_lfoot, state_rfoot ])
        else:
            state = np.array([ imu_pitch, imu_roll])

        return state

    def update_COM(self, cob_x):
        # default_cob_x = -0.015
        # cob_x = -0.05 #-0.6 #-0.06 #-0.02 # -0.015 -0.1

        balance_dict = {
            "updating_duration"                     : 2.0*1.0,

            ####### cob_offset #######
            "cob_x_offset_m"                        : cob_x, #-0.015
            "cob_y_offset_m"                        : -0.00, #-0.00

            ####### FeedForward #####
            "hip_roll_swap_angle_rad"               : 0.00,
            
            ########## Gain ########
            # by gyro
            "foot_roll_gyro_p_gain"                 : 0.5,   #0.25,
            "foot_roll_gyro_d_gain"                 : 0.00,
            "foot_pitch_gyro_p_gain"                : 0.5,   #0.25,
            "foot_pitch_gyro_d_gain"                : 0.00,

            # by imu
            "foot_roll_angle_p_gain"                : 1.0,   #0.35,
            "foot_roll_angle_d_gain"                : 0.1,   #0.00,
            "foot_pitch_angle_p_gain"               : 1.0,   #0.25,
            "foot_pitch_angle_d_gain"               : 0.1,   #0.00,

            # by ft sensor
            "foot_x_force_p_gain"                   : 0.1,   #0.025,
            "foot_x_force_d_gain"                   : 0.00,
            "foot_y_force_p_gain"                   : 0.1,   #0.025,
            "foot_y_force_d_gain"                   : 0.00,
            "foot_z_force_p_gain"                   : 0.02,  #0.001,
            "foot_z_force_d_gain"                   : 0.00,
            
            "foot_roll_torque_p_gain"               : 0.0015, #0.0006,
            "foot_roll_torque_d_gain"               : 0.00,
            "foot_pitch_torque_p_gain"              : 0.0015, #0.0003,
            "foot_pitch_torque_d_gain"              : 0.00,

            ########## CUT OFF FREQUENCY ##########
            # by gyro
            "roll_gyro_cut_off_frequency"           : 50.0,   #40.0,
            "pitch_gyro_cut_off_frequency"          : 50.0,   #40.0,
            "roll_angle_cut_off_frequency"          : 50.0,   #40.0,
            "pitch_angle_cut_off_frequency"         : 50.0,   #40.0,
            "foot_x_force_cut_off_frequency"        : 40.0,   #20.0,
            "foot_y_force_cut_off_frequency"        : 40.0,   #20.0,
            "foot_z_force_cut_off_frequency"        : 40.0,   #20.0,
            "foot_roll_torque_cut_off_frequency"    : 40.0,   #20.0,
            "foot_pitch_torque_cut_off_frequency"   : 40.0    #20.0
        }

        rospy.loginfo('[Env] COM_X: {}'.format(cob_x))
        self.walking.set_balance_param(balance_dict)

    def select_action(self, action):
        # update COM

        if action == 0: # increment
            self.cob_x += self.step_size
        elif action == 1: # decrement
            self.cob_x -= self.step_size
        else: # stop
            pass

        # if self.cob_x < -0.1:
        #     self.cob_x = -0.1
        #     apply_com = False
        
        self.update_COM(self.cob_x)
        self.wait_robot(self.walking, "Balance_Param_Setting_Finished")
        return self.cob_x

    def step(self, action):
        # select action
        cob_x = self.select_action(action)

        # state (IMU + Binarized F/T Sensor)
        state   = self.get_state()
        # rospy.loginfo('[Env] State: {}'.format(state))

        done    = False
        if self.walk_finished:
            done = True

        return state, done, cob_x