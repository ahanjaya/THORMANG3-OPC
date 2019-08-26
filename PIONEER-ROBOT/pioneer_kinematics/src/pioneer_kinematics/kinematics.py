#!/usr/bin/env python

import tf
import rospy
import numpy as np 
from time import sleep
from multipledispatch import dispatch
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from thormang3_manipulation_module_msgs.srv import GetKinematicsPose, GetJointPose
from thormang3_manipulation_module_msgs.msg import KinematicsPose, JointPose

class Kinematics:
    def __init__(self):
        # rospy.init_node('pioneer_kinematics', anonymous=False)
        self.pi  =  3.1415
        self.min = 0
        self.max = 10
        self.xp  = [self.min, self.max]
        self.fp  = [-self.pi, self.pi]

        self.pub_rate       = rospy.Rate(10)
        self.thread_rate    = rospy.Rate(60)

        ## Publisher
        self.module_control_preset_pub = rospy.Publisher('/robotis/enable_ctrl_module', String, queue_size=10) #, latch=True)
        self.send_ini_pose_msg_pub     = rospy.Publisher('/robotis/manipulation/ini_pose_msg', String, queue_size=10) #, latch=True)
        self.send_ik_msg_pub           = rospy.Publisher('/robotis/manipulation/kinematics_pose_msg', KinematicsPose, queue_size=10) #, latch=True)
        self.send_des_joint_msg_pub_   = rospy.Publisher('/robotis/manipulation/joint_pose_msg', JointPose, queue_size=10) 
        self.set_joint_pub             = rospy.Publisher('/robotis/set_joint_states', JointState,    queue_size=10) #, latch=True)

        ## Service Client
        self.get_kinematics_pose_client = rospy.ServiceProxy('/robotis/manipulation/get_kinematics_pose', GetKinematicsPose)
        self.get_joint_pose_client      = rospy.ServiceProxy('/robotis/manipulation/get_joint_pose', GetJointPose)

    def latching_publish(self, topic, msg):
        for i in range(5):
            topic.publish(msg)
            self.pub_rate.sleep()

    def get_kinematics_pose(self, group_name):
        rospy.wait_for_service('/robotis/manipulation/get_kinematics_pose')
        try:
            resp = self.get_kinematics_pose_client(group_name)
            quaternion = ( resp.group_pose.orientation.x, resp.group_pose.orientation.y,
                           resp.group_pose.orientation.z, resp.group_pose.orientation.w )
            euler_rad = tf.transformations.euler_from_quaternion(quaternion) # radian
            euler_deg = np.degrees(euler_rad)
            return {'x'     : resp.group_pose.position.x,
                    'y'     : resp.group_pose.position.y,
                    'z'     : resp.group_pose.position.z,              
                    'roll'  : euler_deg[0],
                    'pitch' : euler_deg[1],
                    'yaw'   : euler_deg[2] }
        except rospy.ServiceException, e:
            print ("Service call failed: %s" %e)

    def set_kinematics_pose(self, group_name, **data):
        msg                 = KinematicsPose()
        msg.name            = group_name
        msg.pose.position.x = data.get('x')
        msg.pose.position.y = data.get('y')
        msg.pose.position.z = data.get('z')

        roll    = np.radians( data.get('roll') )
        pitch   = np.radians( data.get('pitch') )
        yaw     = np.radians( data.get('yaw') )
        quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)

        msg.pose.orientation.x = quaternion[0]
        msg.pose.orientation.y = quaternion[1]
        msg.pose.orientation.z = quaternion[2]
        msg.pose.orientation.w = quaternion[3]
        
        self.latching_publish(self.send_ik_msg_pub, msg)

    def get_joint_pose(self, joint_name):
        rospy.wait_for_service('/robotis/manipulation/get_joint_pose')
        try:
            resp = self.get_joint_pose_client(joint_name)
            return np.degrees(resp.joint_value)
        except rospy.ServiceException, e:
            print ("Service call failed: %s" %e)

    def set_joint_pose(self, joint_name, joint_value):
        msg       = JointPose()
        msg.name  = joint_name
        msg.value = np.radians(joint_value)
        self.latching_publish(self.send_des_joint_msg_pub_, msg)

    def limiter(self, value):
        if value >= self.max:
            return self.max
        elif value <= self.min:
            return self.min
        else:
            return value

    @dispatch(str, int, int)
    def set_gripper(self, group_name, group_value, thumb_y_value):
        group_value   = self.limiter(group_value)
        thumb_y_value = self.limiter(thumb_y_value)
        
        if group_name == "left_arm":
            joint           = JointState()
            joint.name      = ['l_arm_thumb_p', 'l_arm_index_p', 'l_arm_middle_p', 'l_arm_finger45_p']
            joint.position  = [ np.interp(group_value, self.xp, self.fp)  for _ in range(len(joint.name))]
            
            joint.name.append('l_arm_thumb_y')
            joint.position.append( np.interp(thumb_y_value, self.xp, self.fp) )
            
            joint.velocity  = [ 0 for _ in range(len(joint.name)+1)]
            joint.effort    = [ 0 for _ in range(len(joint.name)+1)]
            self.latching_publish(self.set_joint_pub, joint)
            rospy.loginfo("[Kinematics] {0} gripper: {1}, thumb_yaw: {2}".format(group_name, group_value, thumb_y_value))

        elif group_name == "right_arm":
            joint           = JointState()
            joint.name      = ['r_arm_thumb_p', 'r_arm_index_p', 'r_arm_middle_p', 'r_arm_finger45_p']
            joint.position  = [ np.interp(group_value, self.xp, self.fp)  for _ in range(len(joint.name))]
            
            joint.name.append('r_arm_thumb_y')
            joint.position.append( np.interp(thumb_y_value, self.xp, self.fp) )
            
            joint.velocity  = [ 0 for _ in range(len(joint.name)+1)]
            joint.effort    = [ 0 for _ in range(len(joint.name)+1)]
            self.latching_publish(self.set_joint_pub, joint)
            rospy.loginfo("[Kinematics] {0} gripper: {1}, thumb_yaw: {2}".format(group_name, group_value, thumb_y_value))

        else:
            rospy.logerr("[Kinematics] Set gripper: {0} unknown name".format(joint_name))

    @dispatch(list, list)
    def set_gripper(self, joint_name, joint_pose_deg):
        if len(joint_name) == len(joint_pose_deg):       
            joint           = JointState()
            joint.name      = joint_name
            joint.position  = np.radians(joint_pose_deg)
            joint.velocity  = [ 0 for _ in range(len(joint.name))]
            joint.effort    = [ 0 for _ in range(len(joint.name))]
            self.latching_publish(self.set_joint_pub, joint)
            rospy.loginfo('[Kinematics] Gripper joint name: {0} \t Pos: {1}'.format(joint.name, joint.position))
        else:
            rospy.logerr("[Kinematics] Gripper joint_name and joint_pose are not equal")

if __name__ == '__main__':
    k = Kinematics()

    # rospy.loginfo("Set manipulation module")
    # k.latching_publish(k.module_control_preset_pub, "manipulation_module")
    
    # sleep(3)
    # rospy.loginfo("Manipulation init")
    # k.latching_publish(k.send_ini_pose_msg_pub, "ini_pose")

    # sleep(5)
    # # rospy.loginfo("left")
    # left_arm = k.get_kinematics_pose("left_arm")
    # print(left_arm)

    # right_arm = k.get_kinematics_pose("right_arm")
    # print(right_arm)

    # k.set_kinematics_pose("left_arm", **{ 'x': 0.300, 'y': 0.310, 'z': 0.799, 'roll': -0.017, 'pitch': 0.013, 'yaw': 0.007 })
    # k.set_kinematics_pose("right_arm", **{ 'x': 0.300, 'y': -0.277, 'z': 0.756, 'roll': 119.749, 'pitch': 15.072, 'yaw': 9.325 })

    # j = k.get_joint_pose("l_arm_sh_p1")
    # print(j)

    # k.set_joint_pose("l_arm_sh_p1", 50)


    # rospy.loginfo("Set gripper module")
    # k.latching_publish(k.module_control_preset_pub, "gripper_module")
