#!/usr/bin/env python3

import yaml
import rospy
import rospkg
import numpy as np
from time import sleep
from std_msgs.msg import String
from pioneer_motor.motor import Motor

class Action:
    def __init__(self):
        rospy.init_node('pioneer_action', anonymous=False)

        rospack           = rospkg.RosPack()
        self.motion_path  = rospack.get_path("pioneer_motion") + "/config/thormang3_motion_bin.yaml"
        self.motor = Motor()

        self.pub_rate     = rospy.Rate(10)
        self.main_rate    = rospy.Rate(5)
        self.torque_flag  = True
        self.motion       = {}
        self.motion_list = ['hanjaya', 'mandala']

        ## List Sounds
        self.sounds = { 'bye'       : "/home/ppc/Music/thormang_bear_mp3/bye-bye.mp3", 
                        'clap'      : "/home/ppc/Music/thormang_bear_mp3/clap_please.mp3", 
                        'hello'     : "/home/ppc/Music/thormang_bear_mp3/hello_everyone.mp3", 
                        'welcome'   : "/home/ppc/Music/thormang_bear_mp3/welcome_to_erc_lab.mp3",
                        'introduce' : "/home/ppc/Music/thormang_bear_mp3/my_name_is_thormang_bear.mp3" } 

        self.joint_id_to_name = {  1: "r_arm_sh_p1", 2: "l_arm_sh_p1", 3: "r_arm_sh_r",  4: "l_arm_sh_r", 
                                   5: "r_arm_sh_p2", 6: "l_arm_sh_p2", 7: "r_arm_el_y",  8: "l_arm_el_y",
                                   27: "torso_y",   28: "head_y",     29: "head_p" }

        ## Publisher
        self.play_sound_pub = rospy.Publisher('/play_sound_file', String, queue_size=10)

    def publisher_(self, topic, msg, latch=False):
        if latch:
            for i in range(4):
                topic.publish(msg)
                self.pub_rate.sleep()
        else:
            topic.publish(msg)

    def save_motion(self):
        with open(self.motion_path, 'w') as f:
            yaml.dump(self.motion, f, default_flow_style=False)

    def load_motion(self):
        try:
            with open(self.motion_path, 'r') as f:
                data = yaml.safe_load(f)
                if data != None:
                    self.motion = data
                    print('[Action] Loaded motion')
        except yaml.YAMLError as exc:
            print(exc)

    def run(self):
        rospy.loginfo("[Action] running")
        self.load_motion() 

        motor = self.motor
        motor.publisher_(motor.module_control_pub, "none", latch=True)

        while not rospy.is_shutdown():
            print()
            scan = input('[Action] Input : ')

            if scan == 'q' or scan == 'exit':
                print('[Action] Exit with code 0')
                break

            elif scan == 'off' or scan == 'torque off' :
                joint_name = input('\t Joint name : ')
                print('[Action] Torque off: {}'.format(joint_name))

                if joint_name == 'all':
                    motor.set_joint_states(["all"], False)
                    self.torque_flag = False
                else:
                    joint_id = joint_name.split()
                    try:
                        joint = [ self.joint_id_to_name[int(id)] for id in joint_id ]

                        print('[Action] Torque off: {}'.format(joint))
                        motor.set_joint_states(joint, False)
                        self.torque_flag = False
                    except:
                        print('[Action] Torque invalid input')
                
            elif scan == 'on' or scan == 'torque on' :
                joint_name = input('\t Joint name : ')
                print('[Action] Torque on: {}'.format(joint_name))

                if joint_name == 'all':
                    motor.set_joint_states(["all"], True)
                    self.torque_flag = True
                else:
                    joint_id = joint_name.split()
                    try:
                        joint = [ self.joint_id_to_name[int(id)] for id in joint_id ]

                        print('[Action] Torque on: {}'.format(joint))
                        motor.set_joint_states(joint, True)
                        self.torque_flag = True
                    except:
                        print('[Action] Torque invalid input')
           
            elif scan == 's' or scan == 'save':
                if self.torque_flag:
                    motion_name = input('\t Motion name : ')
                    motion_name = motion_name.split()

                    try:
                        header_motion = motion_name[0]
                        sub_motion    = int(motion_name[1])

                        if header_motion not in self.motion:
                            self.motion[header_motion] = dict()

                        self.motion[header_motion][sub_motion] = motor.joint_position
                        self.save_motion()
                        print('[Action] Saved motion: {} '.format(motion_name))
                    except:
                        print('[Action] Please input motion with number')
                else:
                    print('[Action] Failed to save motion, Please turn on torque first')

            elif scan == 'p' or scan == 'play':
                motion_name = input('\t Motion name : ')
                print('[Action] Play motion: {}'.format(motion_name))
                
                motion_name = motion_name.split()

                if len(motion_name) == 2:
                    try:
                        header_motion = motion_name[0]
                        sub_motion    = int(motion_name[1])
                        temp_motion   = self.motion[header_motion][sub_motion]

                        print('\nMotion {} {}'.format(header_motion, sub_motion))
                        print( temp_motion )

                        joint_names    = list(temp_motion.keys())
                        joint_position = list(temp_motion.values())
                        joint_velocity = [ 15 for _ in joint_names ]  # velocity unit in (0%-100%)
                        joint_effort   = [ 20 for _ in joint_names ]  # torque unit in (0%-100%) # 15
                        motor.set_joint_states(joint_names, joint_position, joint_velocity, joint_effort)
                    
                    except:
                        print('[Action] Unknown motion')

                elif len(motion_name) == 1:
                    motion_name = motion_name[0]

                    if motion_name in self.motion:
                        for sub_motion in self.motion[motion_name]:
                            temp_motion = self.motion[motion_name][sub_motion]
                            
                            print('\nMotion {} {}'.format(motion_name, sub_motion))
                            print( temp_motion )
                            
                            joint_names    = list(temp_motion.keys())
                            joint_position = list(temp_motion.values())
                            joint_velocity = [ 15 for _ in joint_names ]
                            joint_effort   = [ 20 for _ in joint_names ]
                            motor.set_joint_states(joint_names, joint_position, joint_velocity, joint_effort)
                            sleep(3)

                # elif motion_name == 'all':
                #     for idx, name in enumerate(self.motion_list):
                #         print('\t Playing motion({}): {}'.format(idx, name))
                #         if name in self.motion:
                #             temp_motion    = self.motion[name]
                #             joint_names    = list(temp_motion.keys())
                #             joint_position = list(temp_motion.values())
                #             joint_velocity = [ 15 for _ in joint_names ]  # velocity unit in (0%-100%)
                #             joint_effort   = [ 20 for _ in joint_names ]  # torque unit in (0%-100%) # 15
                #             motor.set_joint_states(joint_names, joint_position, joint_velocity, joint_effort)
                #         else:
                #             print('[Action] Unknown motion')

                #         sleep(3)
                else:
                    print('[Action] Unknown motion')

            elif scan == 'r' or scan == 'read':
                print(motor.joint_position)

            elif scan == 'h' or scan == 'help':
                print('[Action] --h (help)')
                print()
                print("\t----------Pioneer Action Editor-----------")
                print("\t-- Save Motion----------------------- [s]")
                print("\t-- Read Joints ---------------------- [r]")
                print("\t-- Torque Off ----------------------- [off]")
                print("\t-- Torque On ------------------------ [on]")
                print("\t-- Play Motion----------------------- [p]")
                print("\t-- Exit Action Editor --------------- [q]")
                print()

            elif scan == 'set':
                torque_lvl = 30
                motor.set_joint_torque(['all'], torque_lvl)
                print('[Action] Set torque level: {}%'.format(torque_lvl))

                sleep(1)

                velocity_lvl = 30
                motor.set_joint_velocity(['all'], velocity_lvl)
                print('[Action] Set velocity level: {}%'.format(velocity_lvl))

            else:
                print('[Action] Unknown input')

            self.main_rate.sleep()

        motor.kill_threads()

if __name__ == '__main__':
    action = Action()
    action.run()