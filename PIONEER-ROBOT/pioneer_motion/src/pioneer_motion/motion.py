#!/usr/bin/env python3

import yaml
import rospy
import rospkg
import threading
import numpy as np
from multipledispatch import dispatch
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Float64
from robotis_controller_msgs.msg import StatusMsg
from thormang3_head_control_module_msgs.msg import HeadJointPose

class Motion:
    def __init__(self):
        self.rospack        = rospkg.RosPack()
        self.pub_rate       = rospy.Rate(10)
        self.thread_rate    = rospy.Rate(60)
        self.module_name    = None
        self.status_msg     = None
        self.thread1_flag   = False

        self.init_joint = []
        self.init_pose  = []

        ## Publisher
        self.module_control_pub       = rospy.Publisher('/robotis/enable_ctrl_module',                 String,        queue_size=10) #, latch=True)
        self.move_lidar_pub           = rospy.Publisher('/robotis/head_control/move_lidar',            String,        queue_size=0) #, latch=True)
        self.move_lidar_range_pub     = rospy.Publisher('/robotis/head_control/move_lidar_with_range', Float64,       queue_size=0) #, latch=True)
        self.set_head_joint_pub_      = rospy.Publisher("/robotis/head_control/set_joint_states",      JointState,    queue_size=0) #, latch=True)
        self.set_head_joint_time_pub_ = rospy.Publisher("/robotis/head_control/set_joint_states_time", HeadJointPose, queue_size=0) #, latch=True)

        self.read_robot_status()

    def kill_threads(self):
        self.thread1_flag = True

    def thread_read_robot_status(self, stop_thread):
        while True:
            ## Subscriber
            rospy.Subscriber('/robotis/status', StatusMsg, self.robot_status_callback)
            self.thread_rate.sleep()
            if stop_thread():
                rospy.loginfo("[Motion] Thread killed")
                break

    def robot_status_callback(self, msg):
        self.module_name = msg.module_name
        self.status_msg  = msg.status_msg
        # rospy.loginfo(self.status_msg)

    def read_robot_status(self):
        thread1 = threading.Thread(target = self.thread_read_robot_status, args =(lambda : self.thread1_flag, )) 
        thread1.start()

    def init_motion(self):
        # rospy get package path
        init_pose_path = self.rospack.get_path("pioneer_motion") + "/config/init_pose.yaml"
        init_dict = yaml.load(open(init_pose_path)).get('tar_pose')

        self.init_joint = [ key for key in init_dict ]
        self.init_pose  = [ np.round( np.radians(val), 4 ) for val in init_dict.values() ]

        # print(self.init_joint)
        # print(self.init_pose)

    def publisher_(self, topic, msg, latch=False):
        if latch:
            for i in range(4):
                topic.publish(msg)
                self.pub_rate.sleep()
        else:
            topic.publish(msg)

    @dispatch(list, list)
    def set_head_joint_states(self, joint_name, joint_pose_deg):
        r"""
        Set Head Position
        """
        if len(joint_name) == len(joint_pose_deg):
            joint           = JointState()
            joint.name      = joint_name
            joint.position  = np.radians(joint_pose_deg)
            self.publisher_(self.set_head_joint_pub_, joint)
            # rospy.loginfo('Joint name: {0} \t Pos: {1}'.format(joint.name, joint.position))
        else:
            rospy.logerr("[Motion] Length set_joint_states (position) not equal")

    @dispatch(list, list, float)
    def set_head_joint_states(self, joint_name, joint_pose_deg, time):
        r"""
        Set Head Position with time
        """
        if len(joint_name) == len(joint_pose_deg):
            joint                = HeadJointPose()
            joint.mov_time       = time
            joint.angle.name     = joint_name
            joint.angle.position = np.radians(joint_pose_deg)
            self.publisher_(self.set_head_joint_time_pub_, joint)
            # rospy.loginfo('Joint name: {0} \t Pos: {1}'.format(joint.name, joint.position))
        else:
            rospy.logerr("[Motion] Length set_joint_states (position) not equal")


    # def run(self):
    #     rospy.loginfo("Motion")
    #     self.init_motion()
    #     self.publisher_(self.module_control_pub, "head_control_module")
    #     self.publisher_(self.move_lidar_pub, "start")
    #     self.publisher_(self.move_lidar_range_pub, np.radians(scan_offset+1))

# if __name__ == '__main__':
#     motion = Motion()
#     motion.run()