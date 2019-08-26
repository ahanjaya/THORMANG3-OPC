#!/usr/bin/env python

import yaml
import rospy
import rospkg
import numpy as np

class Motion:
    def __init__(self):
        self.rospack = rospkg.RosPack()

        self.init_joint = []
        self.init_pose  = []

    def init_motion(self):
        # rospy get package path
        init_pose_path = self.rospack.get_path("pioneer_motion") + "/config/init_pose.yaml"
        init_dict = yaml.load(open(init_pose_path)).get('tar_pose')

        self.init_joint = [ key for key in init_dict ]
        self.init_pose  = [ np.round( np.radians(val), 4 ) for val in init_dict.values() ]

        # print(self.init_joint)
        # print(self.init_pose)

    # def run(self):
    #     rospy.loginfo("Motion")
    #     self.init_motion()

# if __name__ == '__main__':
#     mtn = Motion()
#     mtn.run()