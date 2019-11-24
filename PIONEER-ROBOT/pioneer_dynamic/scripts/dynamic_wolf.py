#!/usr/bin/env python3

import rospy
from pioneer_dynamic.ddynamic_reconfigure import DDynamicReconfigure

class Dynamic(object):
    def __init__(self):
        rospy.init_node('initial_pose')
        rospy.loginfo("[Dynamic] Pioneer Dynamic Wolf - Running")

        # Add variables (name, description, default value, min, max, edit_method)

        self.r_foot = DDynamicReconfigure("right_foot")
        self.r_foot.add_variable("x",     "",  0.0,   -1.0, 1.0)
        self.r_foot.add_variable("y",     "", -0.093, -1.0, 1.0)
        self.r_foot.add_variable("z",     "", -0.63,  -1.0, 1.0)
        self.r_foot.add_variable("roll",  "",  0.0,   -1.0, 1.0)
        self.r_foot.add_variable("pitch", "",  0.0,   -1.0, 1.0)
        self.r_foot.add_variable("yaw",   "",  0.0,   -1.0, 1.0)

        self.l_foot = DDynamicReconfigure("left_foot")
        self.l_foot.add_variable("x",     "",  0.0,   -1.0, 1.0)
        self.l_foot.add_variable("y",     "",  0.093, -1.0, 1.0)
        self.l_foot.add_variable("z",     "", -0.63,  -1.0, 1.0)
        self.l_foot.add_variable("roll",  "",  0.0,   -1.0, 1.0)
        self.l_foot.add_variable("pitch", "",  0.0,   -1.0, 1.0)
        self.l_foot.add_variable("yaw",   "",  0.0,   -1.0, 1.0)

        self.cob = DDynamicReconfigure("centre_of_body")
        self.cob.add_variable("x",     "",  0.0,  -1.0, 1.0)
        self.cob.add_variable("y",     "",  0.0,  -1.0, 1.0)
        self.cob.add_variable("z",     "",  0.0,  -1.0, 1.0)
        self.cob.add_variable("roll",  "",  0.0,  -1.0, 1.0)
        self.cob.add_variable("pitch", "",  0.0,  -1.0, 1.0)
        self.cob.add_variable("yaw",   "",  0.0,  -1.0, 1.0)

        # self.add_variables_to_self(self.r_foot)
        self.r_foot.start(self.dyn_rec_callback)

        # self.add_variables_to_self(self.l_foot)
        self.l_foot.start(self.dyn_rec_callback)

        # self.add_variables_to_self(self.cob)
        self.cob.start(self.dyn_rec_callback)

    def add_variables_to_self(self, object_item):
        var_names = object_item.get_variable_names()
        for var_name in var_names:
            self.__setattr__(var_name, None)

    def dyn_rec_callback(self, config, level):
        # rospy.loginfo("Received reconf call: " + str(config))
        # # Update all variables
        # var_names = self.ddr.get_variable_names()
        # for var_name in var_names:
        #     self.__dict__[var_name] = config[var_name]
        return config

if __name__ == '__main__':
    dynamic = Dynamic()
    rospy.spin()