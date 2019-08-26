#!/usr/bin/env python

import signal
import sys
import rospy
from time import sleep
from pioneer_motor.motor import Motor
from pioneer_motion.motion import Motion
from pioneer_kinematics.kinematics import Kinematics

def main():
    rospy.init_node('pioneer_main', anonymous=False)
    rospy.loginfo("Pioneer Main - Running")

    # motion = Motion()
    # motion.init_motion()

    # print(motion.init_joint)
    # print(motion.init_pose)

    kinematics = Kinematics()

    kinematics.set_gripper("left_arm", 0, 8)
    # kinematics.set_gripper(['l_arm_index_p', 'l_arm_middle_p', 'l_arm_finger45_p'], [0,0,0])
    sleep(1)
    kinematics.set_gripper("left_arm", 5, 8)
    sleep(5)

    # kinematics.set_gripper(['l_arm_index_p', 'l_arm_middle_p', 'l_arm_finger45_p'], [-180, -180, -180])
    kinematics.set_gripper("left_arm", 0, 0)

    # motor = Motor()
    # sleep(3)
    # rospy.loginfo(motion.init_pose)
    # # motor.set_joint_states(motion.init_joint, motion.init_pose)
    # # sleep(5)
    # motor.kill_threads()

    # motor.set_joint_states(["l_arm_el_y"], [0.0], [0.0], [0.0])
    # motor.set_joint_states(["l_arm_el_y"], [0.5])

    # rate = rospy.Rate(60)
    # while not rospy.is_shutdown():
    #     rospy.loginfo("infinite")
    #     rate.sleep()

    # motor.kill_threads()

if __name__ == '__main__':
    main()
    