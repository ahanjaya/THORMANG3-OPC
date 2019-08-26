#!/usr/bin/env python

import rospy
import threading
from time import sleep
from multipledispatch import dispatch
from sensor_msgs.msg import JointState
from robotis_controller_msgs.msg import SyncWriteItem

class Motor:
    def __init__(self):
        # rospy.init_node('pioneer_motor', anonymous=False)

        self.pub_rate       = rospy.Rate(10)
        self.thread_rate    = rospy.Rate(60)
        self.joint_position = {}
        self.joint_velocity = {}
        self.joint_effort   = {}

        self.goal_position  = {}
        self.goal_velocity  = {}
        self.goal_effort    = {}
        self.thread_rd      = True

        ## Publisher
        self.set_joint_pub  = rospy.Publisher('/robotis/set_joint_states', JointState,    queue_size=10) #, latch=True)
        self.sync_write_pub = rospy.Publisher('/robotis/sync_write_item',  SyncWriteItem, queue_size=10) #, latch=True)

        self.read_dynamixel()

    def kill_threads(self):
        self.thread_rd = False
        
    def thread_read_dynamixel(self, stop_thread):
        while True:
            ## Subscriber
            rospy.Subscriber('/robotis/present_joint_states', JointState, self.present_joint_states_callback)
            rospy.Subscriber('/robotis/goal_joint_states',    JointState, self.goal_joint_states_callback)
            self.thread_rate.sleep()
            if not stop_thread():
                rospy.loginfo("[Motor] Thread killed")
                break

    def read_dynamixel(self):
        thread1 = threading.Thread(target = self.thread_read_dynamixel, args =(lambda : self.thread_rd, )) 
        thread1.start()

    def present_joint_states_callback(self, msg):
        self.joint_position = dict(zip(msg.name, msg.position))
        self.joint_velocity = dict(zip(msg.name, msg.velocity))
        self.joint_effort   = dict(zip(msg.name, msg.effort))
        # print(msg)

    def goal_joint_states_callback(self, msg):
        self.goal_velocity = dict(zip(msg.name, msg.velocity))
        self.goal_effort   = dict(zip(msg.name, msg.effort))

    def latching_publish(self, topic, msg):
        for i in range(5):
            topic.publish(msg)
            self.pub_rate.sleep()

    @dispatch(list, list)
    def set_joint_states(self, joint_name, joint_pose_deg):
        '''
        Set Position
        '''
        if len(joint_name) == len(joint_pose_deg):
            joint           = JointState()
            joint.name      = joint_name
            joint.position  = np.radians(joint_pose_deg)
            joint.velocity  = [self.goal_velocity.get(_) for _ in joint_name]
            joint.effort    = [self.goal_effort.get(_)   for _ in joint_name]

            self.latching_publish(self.set_joint_pub, joint)
            print('Joint name: {0} \t Pos: {1}'.format(joint.name, joint.position))
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
            joint.effort    = [self.goal_effort.get(_) for _ in joint_name]

            self.latching_publish(self.set_joint_pub, joint)
            print('Joint name: {0} \t Pos: {1} \t Speed: {2}'.format(joint.name, joint.position, joint.velocity))
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

            self.latching_publish(self.set_joint_pub, joint)
            print('Joint name: {0} \t Pos: {1} \t Speed: {2} \t Torque: {3}'.format(joint.name, joint.position, joint.velocity, joint.effort))
        else:
            rospy.logerr("[Motor] Length set_joint_states (position, speed, torque) not equal")

    @dispatch(list, bool)
    def set_joint_states(self, joint_name, torque):
        '''
        Enable/Disable Torque
        '''
        sync_write = SyncWriteItem()
        sync_write.item_name    = "torque_enable"
        sync_write.joint_name   = joint_name
        sync_write.value        = [torque for _ in range(len(joint_name))]

        # turn off torque
        if not torque:
            self.latching_publish(self.sync_write_pub, sync_write)
        # turn on torque
        else:             
            joint           = JointState()
            joint.name      = joint_name
            joint.position  = [self.joint_position.get(_) for _ in joint_name] # read present position
            joint.velocity  = [self.goal_velocity.get(_) for _ in joint_name]
            # joint.effort    = [0 for _ in range(len(joint_name))]
            joint.effort    = [self.goal_effort.get(_) for _ in joint_name]

            self.latching_publish(self.set_joint_pub, joint)        # set present position
            self.latching_publish(self.sync_write_pub, sync_write)  # turn on torque

        print('Joint name: {0} \t Torque: {1}'.format(joint_name, sync_write.value))

    # def run(self):
        # rospy.spin()
        # self.rate = rospy.Rate(60)
        # while not rospy.is_shutdown():
        #     self.rate.sleep()

# if __name__ == '__main__':
    # m = Motor()
    # m.run()
    # m.set_joint_states(["r_arm_el_y"], [0.0], [0.0], [10.0])

    # import motor
    # m = motor.Motor()
    # m.read_dynamixel()
    # m.thread_rd = False
    # m.set_joint_states(["l_arm_el_y", "r_arm_el_y"], True)
    # m.set_joint_states(["r_arm_el_y", "l_arm_el_y"], [0.0], [2.0])