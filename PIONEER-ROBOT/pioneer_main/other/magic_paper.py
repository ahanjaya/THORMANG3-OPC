#!/usr/bin/env python3

import yaml
import rospy
import rospkg
from time import sleep
from std_msgs.msg import String
from pioneer_motor.motor import Motor

class Paper:
    def __init__(self, robot_name):
        self.robot_name = robot_name
        self.motor      = Motor(self.robot_name)
        self.main_rate  = rospy.Rate(10)

    def run(self):
        motor = self.motor
        motor.publisher_(motor.module_control_pub, "none", latch=True)

        while not rospy.is_shutdown():
            text = input('Please input: ')

            if text == 'init':
                motor.publisher_(motor.init_pose_pub, "ini_pose", latch=False)

            elif text == 'off':
                motor.publisher_(motor.module_control_pub, "none", latch=True)
                motor.set_joint_states(['all'], False)
                rospy.loginfo('[Magic] Turn off torque')

            elif text == 'on':
                # motor.publisher_(motor.module_control_pub, "none", latch=True)
                motor.set_joint_states(['all'], True)
                rospy.loginfo('[Magic] Turn on torque')

            elif text == 'q':
                break

            self.main_rate.sleep()
                

if __name__ == "__main__":
    rospy.init_node('wolf_magic', anonymous=False)

    paper = Paper("Thormang3_Wolf")
    paper.run()