#!/usr/bin/env python3

import yaml
import rospy
import rospkg
import numpy as np
from time import sleep
from std_msgs.msg import Bool
from geometry_msgs.msg import Point32
from pioneer_kinematics.kinematics import Kinematics

def wait_robot(obj, msg):
    sleep(1)
    while obj.status_msg != msg:
        pass # do nothing

def main():
    rospy.init_node('pioneer_main_typing', anonymous=False)
    rospy.loginfo("Pioneer Main Typing - Running")

    ##################
    ## Kinematics
    ##################
    kinematics = Kinematics()
    kinematics.publisher_(kinematics.module_control_pub, "manipulation_module", latch=True)  # <-- Enable Manipulation mode
    kinematics.publisher_(kinematics.send_ini_pose_msg_pub, "typing_pose",      latch=True)
    kinematics.publisher_(kinematics.en_typing_pub,       True,                 latch=False) # <-- Enable Typing mode
    wait_robot(kinematics, "End Init Trajectory")

    # Typing Init Pose
    kinematics.set_joint_pos(['head_p', 'head_y', 'torso_y'], [30, 0, 0])
    kinematics.set_gripper("left_arm", 0, 0)
    kinematics.set_gripper("right_arm", 0, 0)
    sleep(2)
    
    # Set Finger
    kinematics.set_joint_pos(['l_arm_index_p', 'l_arm_thumb_p'], [-86, -70])
    kinematics.set_joint_pos(['r_arm_index_p', 'r_arm_thumb_p'], [-86, -70])
    rospy.loginfo('Finish Init Head & Hand')

    main_rate = rospy.Rate(20)
    roll_l, pitch_l, yaw_l = 10.0, 0.0, 0.0

    xl, yl, zl = 0, 0, 0
    init = True
    offset = 0.0

    while not rospy.is_shutdown():
        main_rate.sleep()

        if init:
            key = input('Input:')
            if key == 'f':
                # xl, yl, zl = 0.295, 0.265, 0.74 # first row
                xl, yl, zl = 0.275, 0.255, 0.74 # second row
                kinematics.set_kinematics_pose("left_arm" , 2.0, **{ 'x': xl, 'y': yl, 'z': zl, 'roll': roll_l, 'pitch': pitch_l, 'yaw': yaw_l })
            elif key == 'g':
                # zl = 0.715 # first row
                zl = 0.7
                kinematics.set_kinematics_pose("left_arm" , 2.0, **{ 'x': xl, 'y': yl, 'z': zl, 'roll': roll_l, 'pitch': pitch_l, 'yaw': yaw_l })
            elif key == 'o':
                init = False
            elif key == 'q':
                break
        else:
            key = input('Press Enter for next moving:')
            if key == 'q':
                break
            else:
                # xl += 0.002
                yl -= 0.02
                offset += 0.003
                kinematics.set_kinematics_pose("left_arm" , 2.0, **{ 'x': xl, 'y': yl, 'z': zl, 'roll': roll_l, 'pitch': pitch_l, 'yaw': yaw_l })
                wait_robot(kinematics, "End Left Arm Trajectory")

            key = input('Press Enter for typing:')
            if key == 'q':
                break
            else:
                print('zl:', zl)
                zl = zl - 0.025 - offset
                kinematics.set_kinematics_pose("left_arm" , 2.0, **{ 'x': xl, 'y': yl, 'z': zl, 'roll': roll_l, 'pitch': pitch_l, 'yaw': yaw_l })
                wait_robot(kinematics, "End Left Arm Trajectory")
                sleep(1)

                # go up
                zl = 0.74
                kinematics.set_kinematics_pose("left_arm" , 2.0, **{ 'x': xl, 'y': yl, 'z': zl, 'roll': roll_l, 'pitch': pitch_l, 'yaw': yaw_l })
                wait_robot(kinematics, "End Left Arm Trajectory")
                

    kinematics.kill_threads()

if __name__ == '__main__':
    main()
    