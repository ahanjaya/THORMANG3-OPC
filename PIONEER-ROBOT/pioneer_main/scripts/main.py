#!/usr/bin/env python

import signal
import sys
import rospy
from time import sleep
from pioneer_motor.motor import Motor
from pioneer_motion.motion import Motion
from pioneer_sensors.sensor import Sensor
from pioneer_walking.walking import Walking
from pioneer_kinematics.kinematics import Kinematics

def main():
    rospy.init_node('pioneer_main', anonymous=False)
    rospy.loginfo("Pioneer Main - Running")

    ##################
    ## Kinematics
    ##################
    # print("kinemtics")
    # kinematics = Kinematics()
    # kinematics.latching_publish(kinematics.module_control_preset_pub, "manipulation_module")  # <-- Enable Manipulation mode
    # kinematics.latching_publish(kinematics.send_ini_pose_msg_pub, "ini_pose")
    # kinematics.set_gripper("right_arm", 0, 0)
    # kinematics.set_gripper(['l_arm_index_p', 'l_arm_middle_p', 'l_arm_finger45_p'], [0,0,0])
    # sleep(1)
    # kinematics.set_gripper("left_arm", 5, 8)
    # kinematics.set_gripper(['l_arm_index_p', 'l_arm_middle_p', 'l_arm_finger45_p'], [-180, -180, -180])
    # kinematics.set_gripper("left_arm", 0, 0)

    ##################
    ## Walk
    ##################
    # walk = Walking()
    # walk.latching_publish(walk.walking_pub, "ini_pose")
    # sleep(4)
    # print("walking")
    # walk.latching_publish(walk.walking_pub, "set_mode")  # <-- Enable walking mode
    # sleep(4)
    # print("walking balance on")
    # walk.latching_publish(walk.walking_pub, "balance_on")
    # sleep(4)

    # walk.walk_command("forward", 1, 1.0, 0.01, 0.05, 5)
    # while (walk.status_msg == "Walking_Started"):
    #     rospy.loginfo("walking forward 1")
    
    # walk.walk_command("forward", 1, 1.0, 0.1, 0.05, 5)
    # while (walk.status_msg == "Walking_Started"):
    #     rospy.loginfo("walking forward 2")
    
    # walk.walk_command("forward", 1, 1.0, 0.01, 0.05, 5)
    # while (walk.status_msg == "Walking_Started"):
    #     rospy.loginfo("walking forward 3")

    # sleep(5)
    # walk.kill_threads()


    ##################
    ## Sensor
    ##################
    # sensor = Sensor()
    # sleep(5)
    # sensor.kill_threads()


    # motion = Motion()
    # motion.init_motion()

    # print(motion.init_joint)
    # print(motion.init_pose)

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
    # sensor.kill_threads()
    # walk.kill_threads()

if __name__ == '__main__':
    main()
    