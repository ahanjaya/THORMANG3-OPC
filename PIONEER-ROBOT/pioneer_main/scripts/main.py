#!/usr/bin/env python

import signal
import sys
import rospy
from time import sleep
from pioneer_motor.motor import Motor
from pioneer_vision.camera import Camera
from pioneer_motion.motion import Motion
from pioneer_sensors.sensor import Sensor
from pioneer_walking.walking import Walking
from pioneer_kinematics.kinematics import Kinematics

def main():
    # rospy.init_node('pioneer_main', anonymous=False)
    rospy.loginfo("Pioneer Main - Running")

    camera = Camera()

   
    ##################
    ## Kinematics
    ##################
    # kinematics = Kinematics()   
    # kinematics.publisher_(kinematics.module_control_pub, "manipulation_module", latch=True)  # <-- Enable Manipulation mode
    # kinematics.publisher_(kinematics.send_ini_pose_msg_pub, "ini_pose", latch=True)
    # sleep(2)

    # kinematics.set_gripper("left_arm", 0, 5)
    # kinematics.set_gripper("right_arm", 0, 5)
        
    # while not rospy.is_shutdown():
    #     # kinematics.set_kinematics_pose("right_arm", 2.0, **{ 'x': 0.20, 'y': -0.2, 'z': 0.60, 'roll': 90.00, 'pitch': 40.00, 'yaw': 0.00 })
    #     # kinematics.set_kinematics_pose("left_arm", 2.0, **{ 'x': 0.20, 'y': 0.2, 'z': 0.60, 'roll': -90.00, 'pitch': 40.00, 'yaw': 0.00 })
    #     # sleep(3)

    #     kinematics.set_kinematics_pose("right_arm", 2.0, **{ 'x': 0.40, 'y': -0.4, 'z': 0.62, 'roll': 90.00, 'pitch': 40.00, 'yaw': 0.00 })
    #     kinematics.set_kinematics_pose("left_arm", 2.0, **{ 'x': 0.40, 'y': 0.15, 'z': 0.62, 'roll': -90.00, 'pitch': 40.00, 'yaw': 0.00 })
    #     sleep(3)
        

    #     kinematics.set_kinematics_pose("right_arm", 2.0, **{ 'x': 0.40, 'y': -0.33, 'z': 0.62, 'roll': 90.00, 'pitch': 40.00, 'yaw': 0.00 })
    #     kinematics.set_kinematics_pose("left_arm", 2.0, **{ 'x': 0.40, 'y': 0.08, 'z': 0.62, 'roll': -90.00, 'pitch': 40.00, 'yaw': 0.00 })
    #     sleep(3)
    #     kinematics.set_gripper("left_arm", 3, 5)
    #     kinematics.set_gripper("right_arm", 3, 5)

    #     sleep(0.3)
    #     kinematics.set_kinematics_pose("right_arm", 2.0, **{ 'x': 0.30, 'y': -0.2, 'z': 0.8, 'roll': 90.00, 'pitch': 30.00, 'yaw': 0.00 })
    #     kinematics.set_kinematics_pose("left_arm", 2.0, **{ 'x': 0.30, 'y': 0.2, 'z': 0.8, 'roll': -90.00, 'pitch': 30.00, 'yaw': 0.00 })

    #     sleep(6)
    #     kinematics.set_kinematics_pose("right_arm", 2.0, **{ 'x': 0.40, 'y': -0.33, 'z': 0.62, 'roll': 90.00, 'pitch': 40.00, 'yaw': 0.00 })
    #     kinematics.set_kinematics_pose("left_arm", 2.0, **{ 'x': 0.40, 'y': 0.08, 'z': 0.62, 'roll': -90.00, 'pitch': 40.00, 'yaw': 0.00 })

    #     sleep(3)
    #     kinematics.set_gripper("left_arm", 0, 5)
    #     kinematics.set_gripper("right_arm", 0, 5)
    #     kinematics.set_kinematics_pose("right_arm", 2.0, **{ 'x': 0.40, 'y': -0.4, 'z': 0.62, 'roll': 90.00, 'pitch': 40.00, 'yaw': 0.00 })
    #     kinematics.set_kinematics_pose("left_arm", 2.0, **{ 'x': 0.40, 'y': 0.15, 'z': 0.62, 'roll': -90.00, 'pitch': 40.00, 'yaw': 0.00 })

    #     sleep(3)
    #     kinematics.publisher_(kinematics.send_ini_pose_msg_pub, "ini_pose")
    #     break


    ##################
    ## Motor
    ##################     
    # motor = Motor()
    # motor.publisher_(motor.module_control_pub, "none", latch=True)
    # rate = rospy.Rate(60)
    # while not rospy.is_shutdown():
    #     rospy.loginfo("motor")
    #     # motor.set_joint_states(["l_arm_wr_p", "l_arm_wr_y", "r_arm_wr_y", "r_arm_wr_p"], \
    #     #     [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0])
    #     # sleep(2)
    #     # motor.set_joint_states(["l_arm_wr_p", "l_arm_wr_y", "r_arm_wr_y", "r_arm_wr_p"], \
    #     #     [45, 45, 45, 45], [0, 0, 0, 0], [0, 0, 0, 0])
    #     # sleep(2)
    #     # motor.set_joint_states(["l_arm_wr_p", "l_arm_wr_y", "l_arm_wr_r", "r_arm_wr_y", "r_arm_wr_p", "r_arm_wr_r"], False)

    #     # motor.set_joint_states(["l_arm_thumb_y", "l_arm_thumb_p", "l_arm_index_p", "l_arm_middle_p", "l_arm_finger45_p",
    #     #                         "r_arm_thumb_y", "r_arm_thumb_p", "r_arm_index_p", "r_arm_middle_p", "r_arm_finger45_p"], False)
    #     motor.set_joint_states(["all"], False)
    #     rate.sleep()


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


    ##################
    ## Motion
    ##################

    # motion = Motion()
    # motion.init_motion()
    # print(motion.init_joint)
    # print(motion.init_pose)
    # sleep(3)
    # rospy.loginfo(motion.init_pose)
    # # motor.set_joint_states(motion.init_joint, motion.init_pose)
    # # sleep(5)
    # motor.kill_threads()

    # motor.set_joint_states(["l_arm_el_y"], [0.0], [0.0], [0.0])
    # motor.set_joint_states(["l_arm_el_y"], [0.5])

    rate = rospy.Rate(60)
    while not rospy.is_shutdown():
        # rospy.loginfo("infinite")
        # print(camera.source_image)
        rate.sleep()

    camera.kill_threads()
    # motor.kill_threads()
    # sensor.kill_threads()
    # walk.kill_threads()

if __name__ == '__main__':
    main()
    