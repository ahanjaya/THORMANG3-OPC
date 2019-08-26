#!/usr/bin/env python3


import rospy
import signal
import numpy as np
from time import sleep
from geometry_msgs.msg import Pose, PoseArray, Point

def addition(n): 
    return n + n 

def main():
    rospy.init_node('pioneer_main', anonymous=False)
    rospy.loginfo("Pioneer Main - Running")

    send_ik_msg_pub  = rospy.Publisher('/robotis/pose_msg', PoseArray, queue_size=5)

    geo_pose = [ Pose() for _ in range (5) ]

    geo_pose[].position.x = np.linspace(0.0, np.pi, 5)
    
    for i in range(5):
        geo_pose[i].position.x = i
    
    #[ 10 for _ in range (5) ]

    print(geo_pose)


    numbers = (1, 2, 3, 4) 
    result = map(addition, numbers) 

    # print(geo_pose)
    # geo_pose = Pose()

    # geo_pose.position.x = 10
    # geo_pose.position.y = 20
    # geo_pose.position.z = 30

    # pub_rate = rospy.Rate(10)

    # msg = PoseArray()
    # # msg.poses = [geo_pose              for _ in range (5)]
    # msg.poses = geo_pose

    # # rospy.loginfo(msg)


    
    # for i in range (10):
    #     send_ik_msg_pub.publish(msg)
    #     pub_rate.sleep()
    # while not rospy.is_shutdown:
    #     send_ik_msg_pub.publish(geo_pose)
    #     sleep(2)
    #     break
    # rospy.loginfo(geo_pose)

if __name__ == '__main__':
    main()
    