#!/usr/bin/env python3

import rospy
import threading
import numpy as np 
from pioneer_utils.utils import *
from sensor_msgs.msg import Imu, LaserScan
from geometry_msgs.msg import WrenchStamped
from thormang3_imu_3dm_gx4.msg import FilterOutput

class Sensor:
    def __init__(self, robot_name):
        # rospy.init_node('pionee_sensor', anonymous=False)
        self.robot_name        = robot_name
        self.thread1_flag      = False
        self.thread2_flag      = False
        self.thread3_flag      = False
        self.thread_rate       = rospy.Rate(60)

        #########
        ## IMU
        self.imu_filter        = True
        self.imu_ori           = {}
        self.imu_ori_cov       = None
        self.imu_ang_vel       = None
        self.imu_ang_vel_cov   = None
        self.imu_lin_accel     = None
        self.imu_lin_accel_cov = None
        self.imu_bias          = None
        self.imu_bias_cov      = None

        #########
        ## Lidar
        self.lidar_filter   = False
        self.lidar_ranges   = None

        #########
        ## Foot Sensor
        self.left_torque    = None
        self.right_torque   = None

        self.mutex          = threading.Lock()

        self.read_sensor()

    def kill_threads(self):
        self.thread1_flag   = True
        self.thread2_flag   = True
        self.thread3_flag   = True
        
    def thread_read_IMU(self, stop_thread):
        if self.imu_filter:
            rospy.Subscriber('/robotis/sensor/imu/filter', FilterOutput, self.imu_filter_callback)
            rospy.spin()
        else:
            rospy.Subscriber('/robotis/sensor/imu/imu', Imu, self.imu_callback)
            rospy.spin()
        # while True:
        #     ## Subscriber
        #     if self.imu_filter:
        #         rospy.Subscriber('/robotis/sensor/imu/filter', FilterOutput, self.imu_filter_callback)
        #     else:
        #         rospy.Subscriber('/robotis/sensor/imu/imu', Imu, self.imu_callback)

        #     self.thread_rate.sleep()
        #     if stop_thread():
        #         rospy.loginfo("[Sensor] Thread Read IMU Killed")
        #         break

    def thread_read_Lidar(self, stop_thread):
        if self.lidar_filter:
            rospy.Subscriber('/robotis/sensor/scan_filtered', LaserScan, self.lidar_filter_callback)
            rospy.spin()
        else:
            rospy.Subscriber('/robotis/sensor/scan', LaserScan, self.lidar_callback)    
            rospy.spin()

        # while True:
        #     ## Subscriber
        #     if self.lidar_filter:
        #         rospy.Subscriber('/robotis/sensor/scan_filtered', LaserScan, self.lidar_filter_callback)
        #     else:
        #         rospy.Subscriber('/robotis/sensor/scan', LaserScan, self.lidar_callback)    

        #     self.thread_rate.sleep()
        #     if stop_thread():
        #         rospy.loginfo("[Sensor] Thread Read Lidar Killed")
        #         break

    def thread_read_FTSensor(self, stop_thread):
        rospy.Subscriber('/robotis/sensor/ft_left_foot/scaled',  WrenchStamped, self.left_foot_callback)
        rospy.Subscriber('/robotis/sensor/ft_right_foot/scaled', WrenchStamped, self.right_foot_callback)
        rospy.spin()
        
        # while True:
        #     ## Subscriber
        #     rospy.Subscriber('/robotis/sensor/ft_left_foot/scaled',  WrenchStamped, self.left_foot_callback)
        #     rospy.Subscriber('/robotis/sensor/ft_right_foot/scaled', WrenchStamped, self.right_foot_callback)
            
        #     self.thread_rate.sleep()
        #     if stop_thread():
        #         rospy.loginfo("[Sensor] Thread Read FT Sensor Killed")
        #         break

    def read_sensor(self):
        if self.robot_name  == "Thormang3 Wolf" : # Full size Thormang3
            thread1 = threading.Thread(target = self.thread_read_IMU,       args =(lambda : self.thread1_flag, )) 
            thread2 = threading.Thread(target = self.thread_read_FTSensor,  args =(lambda : self.thread3_flag, )) 
            thread1.start()
            thread2.start()

        thread3 = threading.Thread(target = self.thread_read_Lidar,     args =(lambda : self.thread2_flag, )) 
        thread3.start()

    def imu_callback(self, msg):
        self.mutex.acquire()
        euler_rad = quaternion_to_euler( msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w )
        euler_deg = np.degrees(euler_rad)

        self.imu_ori           = { 'roll': euler_deg[0], 'pitch': euler_deg[1], 'yaw' : euler_deg[2] }
        self.imu_ori_cov       = msg.orientation_covariance
        self.imu_ang_vel       = msg.angular_velocity
        self.imu_ang_vel_cov   = msg.angular_velocity_covariance
        self.imu_lin_accel     = msg.linear_acceleration
        self.imu_lin_accel_cov = msg.linear_acceleration_covariance
        self.mutex.release()

    def imu_filter_callback(self, msg):
        self.mutex.acquire()
        euler_rad = quaternion_to_euler( msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w )
        euler_deg = np.degrees(euler_rad)

        self.imu_ori       = { 'roll': euler_deg[0], 'pitch': euler_deg[1], 'yaw' : euler_deg[2] }
        self.imu_ori_cov   = msg.orientation_covariance
        self.imu_bias      = msg.bias
        self.imu_bias_cov  = msg.bias_covariance # **Critical point to consider**
        self.mutex.release()

    def lidar_callback(self, msg):
        self.mutex.acquire()
        self.lidar_ranges  = msg.ranges
        self.mutex.release()

    def lidar_filter_callback(self, msg):
        self.mutex.acquire()
        self.lidar_ranges  = msg.ranges
        # resolution = (msg.angle_max - msg.angle_min) / len(msg.ranges)
        # print("Angle[rad] reading resolution:", resolution)
        self.mutex.release()

    def left_foot_callback(self, msg):
        self.mutex.acquire()
        self.left_torque = msg.wrench.torque
        self.mutex.release()
    
    def right_foot_callback(self, msg):
        self.mutex.acquire()
        self.right_torque = msg.wrench.torque
        self.mutex.release()