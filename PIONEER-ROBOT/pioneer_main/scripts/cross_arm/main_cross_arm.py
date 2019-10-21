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
        rospy.init_node('pioneer_cross_arm') #, disable_signals=True)
        rospy.loginfo("[CA] Pioneer Main Cross Arm- Running")
        
        self.kinematics  = Kinematics()
        self.motion      = Motion()
        self.action      = Action()
        self.main_rate   = rospy.Rate(10)

        self.prev_arm    = None
        self.arm         = None
        self.state       = None
        self.prev_state  = None
        self.shutdown    = False
        self.object      = False
        self.failed      = False

        self.scan_offset = 50
        self.init_head_p = 30

        ## Publisher
        self.play_sound_pub = rospy.Publisher('/play_sound_file',         String,  queue_size=10)
        # self.shutdown_pub   = rospy.Publisher("/pioneer/shutdown_signal", Bool,    queue_size=10)

        ## Subscriber
        rospy.Subscriber("/pioneer/shutdown_signal",           Bool,   self.shutdown_callback)
        rospy.Subscriber("/pioneer/cross_arm/final_decision",  String, self.final_decision_callback)
        rospy.Subscriber("/pioneer/cross_arm/object",          Bool,   self.object_callback)

    def wait_robot(self, obj, msg):
        while obj.status_msg != msg:
            if self.shutdown:
                break
            else:
                pass # do nothing

    def final_decision_callback(self, msg):
        self.arm = msg.data

    def object_callback(self, msg):
        self.object = msg.data

    def shutdown_callback(self, msg):
        self.shutdown = msg.data
        rospy.loginfo("[CA] Shutdown time")
        rospy.signal_shutdown('Exit')

    def play_sound(self, name):
        sounds = {  'left_arm'        : "/home/ppc/Music/thormang_bear_mp3/coin_left_arm.mp3", 
                    'right_arm'       : "/home/ppc/Music/thormang_bear_mp3/coin_right_arm.mp3",
                    'starwars'        : "/home/ppc/Music/cross_arm_iros_2019/starwars_35s.mp3",
                    'oh_sorry'        : "/home/ppc/Music/cross_arm_iros_2019/oh_sorry.mp3",
                    'hello'           : "/home/ppc/Music/cross_arm_iros_2019/hello.mp3",
                    'introduce'       : "/home/ppc/Music/cross_arm_iros_2019/introduce.mp3",
                    'intro1'          : "/home/ppc/Music/cross_arm_iros_2019/intro1.mp3",
                    'intro2'          : "/home/ppc/Music/cross_arm_iros_2019/intro2.mp3",
                    'intro3'          : "/home/ppc/Music/cross_arm_iros_2019/intro3.mp3",
                    'but'             : "/home/ppc/Music/cross_arm_iros_2019/but.mp3",
                    'suspend'         : "/home/ppc/Music/cross_arm_iros_2019/suspend.mp3",
                    'evil_laugh'      : "/home/ppc/Music/cross_arm_iros_2019/evil_laugh.mp3",
                    'magic_capable'   : "/home/ppc/Music/cross_arm_iros_2019/magic_capable.mp3",
                    'volunteer'       : "/home/ppc/Music/cross_arm_iros_2019/volunteer.mp3",
                    'pickup'          : "/home/ppc/Music/cross_arm_iros_2019/pickup.mp3",
                    'ready?'          : "/home/ppc/Music/cross_arm_iros_2019/ready?.mp3",
                    'step1'           : "/home/ppc/Music/cross_arm_iros_2019/step1_put_back.mp3",
                    'step2'           : "/home/ppc/Music/cross_arm_iros_2019/step2_brings_to_front.mp3",
                    'look'            : "/home/ppc/Music/cross_arm_iros_2019/look.mp3",
                    'cross_arm'       : "/home/ppc/Music/cross_arm_iros_2019/cross_arm.mp3",
                    'begin'           : "/home/ppc/Music/cross_arm_iros_2019/begin.mp3",
                    'open_left_arm'   : "/home/ppc/Music/cross_arm_iros_2019/open_left_arm.mp3",
                    'open_right_arm'  : "/home/ppc/Music/cross_arm_iros_2019/open_right_arm.mp3",
                    'closing'         : "/home/ppc/Music/cross_arm_iros_2019/closing.mp3",
                    'proud'           : "/home/ppc/Music/cross_arm_iros_2019/proud.mp3",
                    'sad'             : "/home/ppc/Music/cross_arm_iros_2019/sad.mp3",
                    'sad_titanic'     : "/home/ppc/Music/cross_arm_iros_2019/sad_titanic.mp3",
                    'not_working'     : "/home/ppc/Music/cross_arm_iros_2019/not_working.mp3",
                    'fail1'           : "/home/ppc/Music/cross_arm_iros_2019/fail1.mp3",
                    'fail2'           : "/home/ppc/Music/cross_arm_iros_2019/fail2.mp3",
                    'cheated'         : "/home/ppc/Music/cross_arm_iros_2019/cheated.mp3",
                    'other_volunteer' : "/home/ppc/Music/cross_arm_iros_2019/other_volunteer.mp3"

                } 

        sound      = String()
        sound.data = sounds[name]
        self.play_sound_pub.publish(sound)

    def wait_action(self):
        while not self.action.finish_action:
            pass

    def run(self):
        kinematics = self.kinematics
        motion     = self.motion
        action     = self.action

        action.motor.publisher_(action.motor.module_control_pub, "none", latch=True)
        action.set_init_config()
        sleep(1)

        action.play_motion_thread("standby")
        self.wait_action()
        sleep(1)

        run_robot = input("\nStart Magic Show (y/n)? ")
        if run_robot == 'y':
            # self.state = 'thinking'
            self.state = 'volunteer'
        else:
            self.shutdown = True

        while not rospy.is_shutdown():
            if self.shutdown:
                rospy.loginfo('[CA] Main Cross Arm Exit ...')
                break

            if self.state == 'thinking':
                rospy.loginfo('[CA] Robot State : {}'.format(self.state))
                action.play_motion_thread("thinking")

                sleep(2)
                self.play_sound('starwars') 
                sleep(36) # 36
                self.wait_action()
                self.state = 'surprise'
                
            elif self.state == 'surprise':
                rospy.loginfo('[CA] Robot State : {}'.format(self.state))
                action.play_motion_thread("surprise")

                sleep(1)
                self.play_sound('oh_sorry') 
                self.wait_action()
                sleep(2)
                action.play_motion("standby")
                self.wait_action()
                self.state = 'hello'

            elif self.state == 'hello':
                rospy.loginfo('[CA] Robot State : {}'.format(self.state))
                action.play_motion_thread("hello")

                sleep(2)
                self.play_sound('hello') 
                self.wait_action()
                sleep(2)
                self.state = 'introduce'

            elif self.state == 'introduce':
                rospy.loginfo('[CA] Robot State : {}'.format(self.state))
                action.play_motion_thread("introduce")

                sleep(3)
                self.play_sound('intro1') 
                self.wait_action()
                sleep(1.5)

                action.play_motion_thread("pp")
                sleep(1)
                self.play_sound('intro2') 
                self.wait_action()
                sleep(1)

                action.play_motion_thread("typing")
                self.play_sound('intro3')
                sleep(0.5)
                self.wait_action()
                sleep(2)

                action.play_motion("standby")
                self.wait_action()
                sleep(2)
                self.state = 'exciting'

                break

            elif self.state == 'exciting':
                rospy.loginfo('[CA] Robot State : {}'.format(self.state))
                action.play_motion_thread("exciting")

                sleep(0.5)
                self.play_sound('but')
                self.wait_action()
                sleep(1)

                self.play_sound('suspend')
                sleep(3)
                action.play_motion("standby")
                self.wait_action()
                sleep(2)
                self.state = 'capable'

            elif self.state == 'capable':
                rospy.loginfo('[CA] Robot State : {}'.format(self.state))
                action.play_motion_thread("capable")

                sleep(2)
                self.play_sound('magic_capable') 
                self.wait_action()
                action.play_motion_thread("fisting")
                self.play_sound('evil_laugh') 
                sleep(0.2)
                self.wait_action()
                sleep(4)
                action.play_motion("standby")
                self.wait_action()
                sleep(2)
                self.state = 'volunteer'

            elif self.state == 'volunteer':
                rospy.loginfo('[CA] Robot State : {}'.format(self.state))
                action.play_motion_thread("volunteer")

                sleep(2)
                self.play_sound('volunteer') 
                self.wait_action()
                sleep(5)
                action.play_motion("standby")
                self.wait_action()
                self.state = 'pickup'

            elif self.state == 'pickup':
                rospy.loginfo('[CA] Robot State : {}'.format(self.state))
                action.play_motion_thread("pickup")

                sleep(2)
                self.play_sound('pickup') 
                self.wait_action()
                sleep(5)
                action.play_motion("standby")
                self.wait_action()
                self.play_sound('ready?') 
                sleep(5)
                self.state = 'instructions'

            elif self.state == 'instructions':
                rospy.loginfo('[CA] Robot State : {}'.format(self.state))

                self.play_sound('step1') 
                sleep(9)
                self.action.play_motion_thread("arm_front")
                self.play_sound('step2') 
                self.wait_action()
                sleep(5)
                self.state = 'look'

            elif self.state == 'look':
                rospy.loginfo('[CA] Robot State : {}'.format(self.state))
                action.play_motion_thread("look")

                self.play_sound('look') 
                sleep(0.2)
                self.wait_action()
                sleep(10)
                action.play_motion("standby")
                self.wait_action()
                sleep(1)
                self.state = 'cross_arm'

            elif self.state == 'cross_arm':
                rospy.loginfo('[CA] Robot State : {}'.format(self.state))
                action.play_motion_thread("cross_arm")
                sleep(0.5)
                self.play_sound('cross_arm')
                sleep(0.2)
                self.wait_action()
                sleep(2)
                action.play_motion("standby")
                self.wait_action()
                self.state = 'head_scanning'
            
            elif self.state == 'head_scanning':
                rospy.loginfo('[CA] Robot State : {}'.format(self.state))
                action.set_init_config(0, 0) # set torque & velocity to default 

                motion.publisher_(motion.module_control_pub, "head_control_module", latch=True)
                motion.publisher_(motion.move_lidar_pub, "start") # scan full head_p
                self.play_sound('begin')
                self.wait_robot(motion, "Finish head joint in order to make pointcloud")

                self.state = 'standby_pointing'

            elif self.state == 'standby_pointing':
                rospy.loginfo('[CA] Robot State : {}'.format(self.state))

                while self.arm == None:
                    pass
                kinematics.publisher_(kinematics.module_control_pub, "manipulation_module", latch=True)

                # play sound
                self.play_sound(self.arm)

                if self.arm == "left_arm":
                    kinematics.set_joint_pos(['l_arm_thumb_p', 'l_arm_middle_p', 'l_arm_finger45_p'], [-60, 180, 180])
                    kinematics.set_kinematics_pose("left_arm", 2.0, **{ 'x': 0.4, 'y': 0.1, 'z': 0.80, 'roll': 0.00, 'pitch': -10.00, 'yaw': -5.00 })
                elif self.arm == "right_arm":
                    kinematics.set_joint_pos(['r_arm_thumb_p', 'r_arm_middle_p', 'r_arm_finger45_p'], [-60, 180, 180])
                    kinematics.set_kinematics_pose("right_arm", 2.0, **{ 'x': 0.4, 'y': -0.1, 'z': 0.80, 'roll': 0.00, 'pitch': -10.00, 'yaw': 5.00 })

                sleep(3)

                if self.arm == "left_arm":
                    self.play_sound('open_left_arm')
                elif self.arm == "right_arm":
                    self.play_sound('open_right_arm')

                self.arm = None
                self.state = 'check'

            elif self.state == 'check': # vision
                rospy.loginfo('[CA] Robot State : {}'.format(self.state))
                action.motor.publisher_(action.motor.module_control_pub, "none", latch=True)
                action.set_init_config(torque=50)

                check1 = False
                check2 = False
                sleep(3)

                if self.object:
                    check1 = True
                else:
                    check1 = False
                if check1:
                    self.state = 'success'
                else:
                    if not self.failed:
                        # open another hand
                        self.play_sound('fail1')

                        sleep(6)
                        if self.object:
                            check2 = True
                        else:
                            check2 = False
                        
                        if check2:
                            # if there is coin
                            self.play_sound('fail2')
                            # retry one more
                            self.failed = True
                            # self.state = 'instructions'
                        else:
                            # if there is no coin
                            self.play_sound('cheated')
                            # wait
                            sleep(4)
                            self.play_sound('other_volunteer')
                            sleep(5)
                            # self.state = 'volunteer'
                    else:
                        self.state = 'sad'
 
                # break

            elif self.state == 'success':
                rospy.loginfo('[CA] Robot State : {}'.format(self.state))

                # play sound
                self.play_sound('closing')
                action.play_motion("happy")
                self.wait_action()
                # sleep(1) # wait closing music finish
                action.play_motion("standby")
                self.wait_action()
                sleep(5)
                self.play_sound('proud')

                break

            elif self.state == 'sad':
                rospy.loginfo('[CA] Robot State : {}'.format(self.state))

                # play sound
                # self.play_sound('sad')
                self.play_sound('sad_titanic')
                
                action.play_motion("sad")
                self.wait_action()
                sleep(10)
                action.play_motion("standby")
                self.wait_action()
                self.play_sound('not_working')

                break

            else:
                if self.state != None:
                    rospy.loginfo('[CA] Invalid Robot State : {}'.format(self.state))

            
                
            self.main_rate.sleep()


        # kinematics.kill_threads()
        # motion.kill_threads()
        # action.kill_threads()

if __name__ == '__main__':
    ca = Cross_Arm()
    ca.run()