#!/usr/bin/env python3

import rospy
import rospkg
import numpy as np


joint_id_to_name = {  1: "r_arm_sh_p1", 2: "l_arm_sh_p1", 
                      3: "r_arm_sh_r",  4: "l_arm_sh_r", 
                      5: "r_arm_sh_p2", 6: "l_arm_sh_p2", 
                      7: "r_arm_el_y",  8: "l_arm_el_y",
                     27: "torso_y",    28: "head_y",
                     29: "head_p" }

# print(joint_id_to_name)
# print(joint_id_to_name.keys())
# print(joint_id_to_name.values())

joint_name = input('\t Joint name : ')
joint_id = joint_name.split()

print(joint_id)
try:
    joint = [ joint_id_to_name[int(id)] for id in joint_id  ]
    print(joint)
except:
    print('wrong')
