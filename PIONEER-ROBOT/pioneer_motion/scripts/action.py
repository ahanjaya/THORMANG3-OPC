#!/usr/bin/env python3

import yaml
import rospy
import rospkg
import threading
import numpy as np
from std_msgs.msg import String, Float64

class Action:
    def __init__(self):
        rospy.init_node('pioneer_action', anonymous=False)

        self.pub_rate     = rospy.Rate(10)
        self.main_rate    = rospy.Rate(60)
        # self.thread_rate    = rospy.Rate(60)
        # self.thread1_flag   = False

        ## List Sounds
        self.sounds = { 'bye'       : "/home/ppc/Music/thormang_bear_mp3/bye-bye.mp3", 
                        'clap'      : "/home/ppc/Music/thormang_bear_mp3/clap_please.mp3", 
                        'hello'     : "/home/ppc/Music/thormang_bear_mp3/hello_everyone.mp3", 
                        'welcome'   : "/home/ppc/Music/thormang_bear_mp3/welcome_to_erc_lab.mp3",
                        'introduce' : "/home/ppc/Music/thormang_bear_mp3/my_name_is_thormang_bear.mp3" } 

        ## Publisher
        self.play_sound_pub = rospy.Publisher('/play_sound_file', String, queue_size=10)

    def publisher_(self, topic, msg, latch=False):
        if latch:
            for i in range(4):
                topic.publish(msg)
                self.pub_rate.sleep()
        else:
            topic.publish(msg)

    def run(self):
        rospy.loginfo("[Action] running")

        while not rospy.is_shutdown():
            for i in self.sounds:
                rospy.loginfo("[Action] Playing")

            break
            self.main_rate.sleep()

if __name__ == '__main__':
    action = Action()
    action.run()