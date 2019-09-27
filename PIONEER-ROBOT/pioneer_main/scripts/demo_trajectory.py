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
        kinematics.trajectory_sin(group="left_arm",  x=0.30, y=0.20,  z=0.90, roll=0.0, pitch=0.0, yaw=0.0, xc=-0.1, yc=-0.1, zc=-0.1, time=2, res=0.01)
        kinematics.trajectory_sin(group="right_arm", x=0.30, y=-0.20, z=0.90, roll=0.0, pitch=0.0, yaw=0.0, xc=-0.1, yc=0.1,  zc=-0.1, time=2, res=0.01)
        sleep(1) # because pubslishing array msg using for loop
        while kinematics.left_arr == True and kinematics.right_arr == True:
            pass # no action
        # input("Press enter")

        kinematics.trajectory_sin(group="left_arm",  x=0.20, y=0.30, z=0.85, roll=0.0, pitch=0.0, yaw=0.0, xc=0.1, yc=0.1, zc=0.1, time=2, res=0.01)
        kinematics.trajectory_sin(group="right_arm", x=0.20, y=-0.30, z=0.85, roll=0.0, pitch=0.0, yaw=0.0, xc=0.1, yc=-0.1, zc=0.1, time=2, res=0.01)
        sleep(1) # because pubslishing array msg using for loop

        while kinematics.left_arr == True and kinematics.right_arr == True:
            pass # no action
        # input("Press enter1")
        
        iter += 1
        rospy.loginfo("[Tra] Iteration : {}".format(iter))
        if iter >= 2:
            break

    kinematics.publisher_(kinematics.send_ini_pose_msg_pub, "ini_pose", latch=True)
    kinematics.kill_threads()

if __name__ == '__main__':
    main()
    