#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Bool, Int16

rospy.init_node('pioneer_test')

test_pub  = rospy.Publisher('/hanjaya', Bool,  queue_size=10)
main_rate = rospy.Rate(60)

while not rospy.is_shutdown():
    test_pub.publish(True)
    main_rate.sleep()