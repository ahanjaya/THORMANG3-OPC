#!/usr/bin/env python3

import sys
import rospy
import numpy as np
from time import sleep
from std_msgs.msg import String, Bool
from pioneer_motion.motion import Motion
from pioneer_motion.action import Action
from pioneer_kinematics.kinematics import Kinematics

class Cross_Arm:
    def __init__(self):
        rospy.init_node('pioneer_cross_arm', disable_signals=True)
        rospy.loginfo("[CA] Pioneer Main Cross Arm- Running")
        
        self.kinematics  = Kinematics()
        self.motion      = Motion()
        self.action      = Action()
        self.main_rate   = rospy.Rate(30)

        self.prev_arm    = None
        self.arm         = None
        self.state       = None
        self.prev_state  = None
        self.shutdown    = False

        self.scan_offset = 50
        self.init_head_p = 30

        ## Publisher
        self.play_sound_pub = rospy.Publisher('/play_sound_file',         String,  queue_size=10)
        # self.shutdown_pub   = rospy.Publisher("/pioneer/shutdown_signal", Bool,    queue_size=10)

        ## Subscriber
        rospy.Subscriber("/pioneer/cross_arm/final_decision",  String, self.final_decision_callback)

    # def myhook(self):
    #     self.shutdown_pub.publish(True)
    #     rospy.loginfo("[CA] Shutdown time")

    def wait_robot(self, obj, msg):
        while obj.status_msg != msg:
            if self.shutdown:
                break
            else:
                pass # do nothing

    def final_decision_callback(self, msg):
        self.arm = msg.data

    def play_sound(self, name):
        sounds = { 'left_arm'  : "/home/ppc/Music/thormang_bear_mp3/coin_left_arm.mp3", 
                    'right_arm' : "/home/ppc/Music/thormang_bear_mp3/coin_right_arm.mp3",
                    'bye'       : "/home/ppc/Music/thormang_bear_mp3/bye-bye.mp3", 
                    'clap'      : "/home/ppc/Music/thormang_bear_mp3/clap_please.mp3", 
                    'hello'     : "/home/ppc/Music/thormang_bear_mp3/hello_everyone.mp3", 
                    'welcome'   : "/home/ppc/Music/thormang_bear_mp3/welcome_to_erc_lab.mp3",
                    'introduce' : "/home/ppc/Music/thormang_bear_mp3/my_name_is_thormang_bear.mp3" } 

        sound      = String()
        sound.data = sounds[name]
        self.play_sound_pub.publish(sound)

    def run(self):
        kinematics = self.kinematics
        motion     = self.motion
        action     = self.action

        kinematics.publisher_(kinematics.module_control_pub, "manipulation_module", latch=True)  # <-- Enable Manipulation mode
        rospy.loginfo('[CA] Manipulation start init pose')
        # sleep(1)
        kinematics.publisher_(kinematics.send_ini_pose_msg_pub, "ini_pose", latch=True)
        self.wait_robot(kinematics, "End Init Trajectory")
        rospy.loginfo('[CA] Manipulation finish init pose')

        sleep(1)

        motion.publisher_(motion.module_control_pub, "head_control_module", latch=True)
        motion.set_head_joint_states(['head_y', 'head_p'], [0, self.init_head_p])
        self.wait_robot(motion, "Head movement is finished.")
        rospy.loginfo('[CA] Finish head init')

        # self.state = 'head_down'
        sleep(2)
        self.state = 'hello'

        # input("Proceed to next ? ")

        try:
            while not rospy.is_shutdown():
                if self.state == 'head_scanning':
                    rospy.loginfo('[CA] Robot State : {}'.format(self.state))

                    motion.publisher_(motion.module_control_pub, "head_control_module", latch=True)
                    motion.publisher_(motion.move_lidar_pub, "start") # scan full head_p
                    sleep(1)
                    motion.publisher_(motion.override_original_pos_lidar_pub,  self.init_head_p) # overide original lidar pose
                    # motion.publisher_(motion.move_lidar_range_pub, np.radians(self.scan_offset+1)) # scan head with range

                    self.wait_robot(motion, "Finish head joint in order to make pointcloud")
                    # self.state = None
                    # sleep(5)
                    self.prev_state = self.state
                    self.state      = 'standby_pointing'

                elif self.state == 'standby_pointing':
                    rospy.loginfo('[CA] Robot State : {}'.format(self.state))

                    while self.arm == None:
                        pass
                    kinematics.publisher_(kinematics.module_control_pub, "manipulation_module", latch=True)

                    # play sound
                    self.play_sound(self.arm)

                    if self.arm == "left_arm":
                        kinematics.set_joint_pos(['l_arm_thumb_p', 'l_arm_middle_p', 'l_arm_finger45_p'], [-60, 180, 180])
                        kinematics.set_kinematics_pose("left_arm", 2.0, **{ 'x': 0.4, 'y': 0.1, 'z': 0.80, 'roll': 0.00, 'pitch': -10.00, 'yaw': 15.00 })

                    elif self.arm == "right_arm":
                        kinematics.set_joint_pos(['r_arm_thumb_p', 'r_arm_middle_p', 'r_arm_finger45_p'], [-60, 180, 180])
                        kinematics.set_kinematics_pose("right_arm", 2.0, **{ 'x': 0.4, 'y': -0.1, 'z': 0.80, 'roll': 0.00, 'pitch': -10.00, 'yaw': -15.00 })

                    self.arm   = None
                    # self.state = None
                    sleep(5)
                    self.prev_state = self.state
                    self.state      = 'init_pose'

                elif self.state == 'init_pose':
                    rospy.loginfo('[CA] Robot State : {}'.format(self.state))

                    kinematics.publisher_(kinematics.module_control_pub, "manipulation_module", latch=True)
                    kinematics.publisher_(kinematics.send_ini_pose_msg_pub, "ini_pose", latch=True)
                    kinematics.set_gripper("left_arm", 0, 5)
                    kinematics.set_gripper("right_arm", 0, 5)
                    self.wait_robot(kinematics, "End Init Trajectory")

                    # self.state = None
                    sleep(1)
                    if self.prev_state == 'hello':
                        self.prev_state = self.state
                        self.state      = 'head_down'

                    elif self.prev_state == 'standby_pointing':
                        self.prev_state = self.state
                        self.state      = None

                elif self.state == 'head_down':
                    rospy.loginfo('[CA] Robot State : {}'.format(self.state))

                    motion.publisher_(motion.module_control_pub, "head_control_module", latch=True)
                    motion.set_head_joint_states(['head_y', 'head_p'], [0, 85])
                    self.wait_robot(motion, "Head movement is finished.")

                    # self.state = None
                    sleep(1.5) #3
                    self.prev_state = self.state
                    self.state      = 'head_scanning'

                elif self.state == 'hello':
                    rospy.loginfo('[CA] Robot State : {}'.format(self.state))

                    action.motor.publisher_(action.motor.module_control_pub, "none", latch=True)
                    action.set_init_config()
                    action.play_motion_thread("hello")

                    # play sound
                    sleep(2)
                    self.play_sound('hello')

                    while not action.finish_action:
                        pass

                    # self.state = None
                    self.prev_state = self.state
                    self.state      = 'init_pose'
                else:
                    if self.state != None:
                        rospy.loginfo('[CA] Invalid Robot State : {}'.format(self.state))

                self.main_rate.sleep()

        except KeyboardInterrupt:
            self.shutdown = True

        # rospy.on_shutdown(self.myhook)
        # kinematics.kill_threads()
        # motion.kill_threads()
        # action.kill_threads()

if __name__ == '__main__':
    ca = Cross_Arm()
    ca.run()