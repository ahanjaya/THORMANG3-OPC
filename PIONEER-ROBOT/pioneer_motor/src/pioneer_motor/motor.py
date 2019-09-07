#!/usr/bin/env python3

import rospy
import threading
import numpy as np
from time import sleep
from std_msgs.msg import String
from multipledispatch import dispatch
from sensor_msgs.msg import JointState
from robotis_controller_msgs.msg import SyncWriteItem

class Motor:
    def __init__(self):
        # rospy.init_node('pioneer_motor', anonymous=False)

        self.pub_rate       = rospy.Rate(10)
        self.thread_rate    = rospy.Rate(60) #60
        self.joint_position = {}
        self.joint_velocity = {}
        self.joint_effort   = {}

        self.goal_position  = {}
        self.goal_velocity  = {}
        self.goal_effort    = {}
        self.thread1_flag   = False

        ## publish
        self.module_control_pub = rospy.Publisher('/robotis/enable_ctrl_module', String,        queue_size=10) #, latch=True)
        self.set_joint_pub      = rospy.Publisher('/robotis/set_joint_states',   JointState,    queue_size=10) #, latch=True)
        self.sync_write_pub     = rospy.Publisher('/robotis/sync_write_item',    SyncWriteItem, queue_size=10) #, latch=True)

        self.read_dynamixel()

    def kill_threads(self):
        self.thread1_flag = True
        
    def thread_read_dynamixel(self, stop_thread):
        while True:
            ## Subscriber
            rospy.Subscriber('/robotis/present_joint_states', JointState, self.present_joint_states_callback)
            # rospy.Subscriber('/robotis/goal_joint_states',    JointState, self.goal_joint_states_callback)
            self.thread_rate.sleep()
            if stop_thread():
                rospy.loginfo("[Motor] Thread killed")
                break

    def read_dynamixel(self):
        thread1 = threading.Thread(target = self.thread_read_dynamixel, args =(lambda : self.thread1_flag, )) 
        thread1.start()

    def present_joint_states_callback(self, msg):
        self.joint_position = dict(zip(msg.name, np.degrees(msg.position)))
        self.joint_velocity = dict(zip(msg.name, msg.velocity))
        self.joint_effort   = dict(zip(msg.name, msg.effort))
        # rospy.loginfo(msg)

    def goal_joint_states_callback(self, msg):
        self.goal_velocity = dict(zip(msg.name, msg.velocity))
        self.goal_effort   = dict(zip(msg.name, msg.effort))

    def publisher_(self, topic, msg, latch=False):
        if latch:
            for i in range(4):
                topic.publish(msg)
                self.pub_rate.sleep()
        else:
            topic.publish(msg)

    @dispatch(list, list)
    def set_joint_states(self, joint_name, joint_pose_deg):
        '''
        Set Position
        '''
        if len(joint_name) == len(joint_pose_deg):
            joint           = JointState()
            joint.name      = joint_name
            joint.position  = np.radians(joint_pose_deg)
            joint.velocity  = [ 0 for _ in joint_name ]
            joint.effort    = [ 0 for _ in joint_name ]
            # joint.velocity  = [ self.goal_velocity.get(_) for _ in joint_name ]
            # joint.effort    = [self.goal_effort.get(_)   for _ in joint_name]
            self.publisher_(self.set_joint_pub, joint, latch=False)
            # rospy.loginfo('Joint name: {0} \t Pos: {1}'.format(joint.name, joint.position))
        else:
            rospy.logerr("[Motor] Length set_joint_states (position) not equal")

    @dispatch(list, list, list)
    def set_joint_states(self, joint_name, joint_pose_deg, joint_speed):
        '''
        Set Position and Speed
        Value Speed | Min = -17000, Max = 17000
        '''
        if ( len(joint_name) == len(joint_pose_deg) and \
             len(joint_name) == len(joint_speed) ):

            joint           = JointState()
            joint.name      = joint_name
            joint.position  = np.radians(joint_pose_deg)
            joint.velocity  = joint_speed
            joint.effort    = [ 0 for _ in joint_name ]
            # joint.effort    = [ self.goal_effort.get(_) for _ in joint_name ]
            self.publisher_(self.set_joint_pub, joint)
            # rospy.loginfo('Joint name: {0} \t Pos: {1} \t Speed: {2}'.format(joint.name, joint.position, joint.velocity))
        else:
            rospy.logerr("[Motor] Length set_joint_states (position, speed) not equal")

    @dispatch(list, list, list, list)
    def set_joint_states(self, joint_name, joint_pose_deg, joint_speed, joint_torque):
        '''
        Set Position, Speed, Torque
        Value Torque | Min = 0, Max = 310
        '''
        if ( len(joint_name) == len(joint_pose_deg)  and \
             len(joint_name) == len(joint_speed) and \
             len(joint_name) == len(joint_torque) ):

            joint           = JointState()
            joint.name      = joint_name
            joint.position  = np.radians(joint_pose_deg)
            joint.velocity  = joint_speed
            joint.effort    = joint_torque
            self.publisher_(self.set_joint_pub, joint)
            # rospy.loginfo('Joint name: {0} \t Pos: {1} \t Speed: {2} \t Torque: {3}'.format(joint.name, joint.position, joint.velocity, joint.effort))
        else:
            rospy.logerr("[Motor] Length set_joint_states (position, speed, torque) not equal")

    @dispatch(list, bool)
    def set_joint_states(self, joint_name, torque):
        '''
        Enable/Disable Torque
        '''
        sync_write = SyncWriteItem()
        sync_write.item_name    = "torque_enable"

        if joint_name[0] == "all":
            sync_write.joint_name   = ["head_p", "head_y", "torso_y", 
                                        "l_arm_el_y", "l_arm_sh_p1", "l_arm_sh_p2", "l_arm_sh_r", "l_arm_wr_p", "l_arm_wr_r", "l_arm_wr_y", 
                                        "r_arm_el_y", "r_arm_sh_p1", "r_arm_sh_p2", "r_arm_sh_r", "r_arm_wr_p", "r_arm_wr_r", "r_arm_wr_y", 
                                        "l_arm_thumb_y", "l_arm_thumb_p", "l_arm_index_p", "l_arm_middle_p", "l_arm_finger45_p",
                                        "r_arm_thumb_y", "r_arm_thumb_p", "r_arm_index_p", "r_arm_middle_p", "r_arm_finger45_p"]
        else:
            sync_write.joint_name   = joint_name
        sync_write.value        = [ torque for _ in range(len(joint_name)) ]

        # turn off torque
        if not torque:
            self.publisher_(self.sync_write_pub, sync_write)
        # turn on torque
        else:             
            joint           = JointState()
            joint.name      = joint_name
            joint.position  = [ self.joint_position.get(_) for _ in joint_name ] # read present position
            joint.velocity  = [ self.goal_velocity.get(_) for _ in joint_name ]
            joint.effort    = [ 0 for _ in range(len(joint_name)) ]

            self.publisher_(self.set_joint_pub, joint)        # set present position
            self.publisher_(self.sync_write_pub, sync_write)  # turn on torque

        # rospy.loginfo('Joint name: {0} \t Torque: {1}'.format(joint_name, sync_write.value))