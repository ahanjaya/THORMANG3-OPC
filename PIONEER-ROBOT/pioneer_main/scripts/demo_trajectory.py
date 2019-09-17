#!/usr/bin/env python3

import rospy
from time import sleep
from pioneer_motor.motor import Motor
from pioneer_kinematics.kinematics import Kinematics

def main():
    rospy.init_node('pioneer_trajectory', anonymous=False)
    rospy.loginfo("[Tra] Pioneer Demo Trajectory - Running")

    ## Trajectory
    kinematics = Kinematics()
    kinematics.publisher_(kinematics.module_control_pub, "manipulation_module", latch=True)  # <-- Enable Manipulation mode
    kinematics.publisher_(kinematics.send_ini_pose_msg_pub, "ini_pose", latch=True)
    sleep(2)

    # trajectory simulation circle
    kinematics.set_kinematics_pose("left_arm" , 2.0, **{ 'x': 0.20, 'y':  0.30, 'z': 0.80, 'roll': 0.00, 'pitch': 0.00, 'yaw': 0.00 })
    kinematics.set_kinematics_pose("right_arm" , 2.0, **{ 'x': 0.20, 'y':  -0.30, 'z': 0.80, 'roll': 0.00, 'pitch': 0.00, 'yaw': 0.00 })

    sleep(0.1)
    while kinematics.left_tra == True and kinematics.right_tra == True:
        pass
    sleep(1)
 
    iter = 0

    while not rospy.is_shutdown():
        kinematics.trajectory_sin(group="left_arm",  x=0.30, y=0.20, z=0.90, roll=0.0, pitch=0.0, yaw=0.0, xc=-0.1, yc=-0.1, zc=-0.1, time=2, res=0.01)
        kinematics.trajectory_sin(group="right_arm", x=0.30, y=-0.20, z=0.90, roll=0.0, pitch=0.0, yaw=0.0, xc=-0.1, yc=0.1, zc=-0.1, time=2, res=0.01)
        sleep(1) # because pubslishing array msg using for loop
        while kinematics.left_arr == True and kinematics.right_arr == True:
            pass # no action
        # input("Press enter")

        kinematics.trajectory_sin(group="left_arm",  x=0.20, y=0.30, z=0.80, roll=0.0, pitch=0.0, yaw=0.0, xc=0.1, yc=0.1, zc=0.1, time=2, res=0.01)
        kinematics.trajectory_sin(group="right_arm",  x=0.20, y=-0.30, z=0.80, roll=0.0, pitch=0.0, yaw=0.0, xc=0.1, yc=-0.1, zc=0.1, time=2, res=0.01)
        sleep(1) # because pubslishing array msg using for loop

        while kinematics.left_arr == True and kinematics.right_arr == True:
            pass # no action
        # input("Press enter1")
        
        iter += 1
        
        rospy.loginfo("[Tra] Iteration : {}".format(iter))
        
        if iter >= 2:
            break


    # while not rospy.is_shutdown():
    #     kinematics.set_gripper("left_arm", 2, 5)
    #     kinematics.set_gripper("right_arm", 2, 5)
    #     # sleep(2)
    #     break

    # left_arm = kinematics.get_kinematics_pose("left_arm")
    # right_arm = kinematics.get_kinematics_pose("right_arm")
    # sleep(1)

    # while not rospy.is_shutdown():
    #     kinematics.set_kinematics_pose("left_arm" , 2.0, **{ 'x': left_arm.get('x'), 'y':  left_arm.get('y'), \
    #                                 'z': left_arm.get('z') + 0.1, 'roll': left_arm.get('roll'), 'pitch': left_arm.get('pitch'), 'yaw': left_arm.get('yaw') })
    #     kinematics.set_kinematics_pose("right_arm" , 2.0, **{ 'x': right_arm.get('x'), 'y':  right_arm.get('y'), \
    #     'z': right_arm.get('z') + 0.1, 'roll': right_arm.get('roll'), 'pitch': right_arm.get('pitch'), 'yaw': right_arm.get('yaw') })
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

    kinematics.kill_threads()
    # motor.kill_threads()

if __name__ == '__main__':
    main()
    