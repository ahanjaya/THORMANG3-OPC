#! /usr/bin/env python3

import rospy
import numpy as np
import matplotlib.pyplot as plt
from std_msgs.msg import Float32
from pioneer_sensors.sensor import Sensor

class Sensor_Util:
    def __init__(self):
        self.sensor        = Sensor("Thormang3_Wolf")
       
        # rqt_plot
        self.imu_roll_pub  = rospy.Publisher('/pioneer/dragging/imu_roll',  Float32,  queue_size=1)
        self.imu_pitch_pub = rospy.Publisher('/pioneer/dragging/imu_pitch', Float32,  queue_size=1)
        self.imu_yaw_pub   = rospy.Publisher('/pioneer/dragging/imu_yaw',   Float32,  queue_size=1)
        
        self.lfoot_pub     = rospy.Publisher('/pioneer/dragging/lfoot',     Float32,  queue_size=1)
        self.rfoot_pub     = rospy.Publisher('/pioneer/dragging/rfoot',     Float32,  queue_size=1)

    def run(self):
        sensor = self.sensor

        while not rospy.is_shutdown():
            # IMU
            self.imu_roll_pub.publish(sensor.imu_ori['roll'])
            self.imu_pitch_pub.publish(sensor.imu_ori['pitch'])
            self.imu_yaw_pub.publish(sensor.imu_ori['yaw'])

            # FT Sensor
            try:
                left_foot  = np.array([ sensor.left_torque['x'],  sensor.left_torque['y'],  sensor.left_torque['z']  ])
                right_foot = np.array([ sensor.right_torque['x'], sensor.right_torque['y'], sensor.right_torque['z'] ])
                origin     = np.array([0, 0, 0])

                euc_lfoot  = np.linalg.norm(left_foot - origin)
                euc_rfoot  = np.linalg.norm(right_foot - origin)

                self.lfoot_pub.publish(euc_lfoot)
                self.rfoot_pub.publish(euc_rfoot)
            
            except:
                pass

        # rospy.spin()

if __name__ == "__main__":
    rospy.init_node('pioneer_sensor_utils')
    utiliity = Sensor_Util()
    utiliity.run()