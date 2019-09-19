#!/usr/bin/env python3

import pcl
import pptk
import ros_numpy
import sensor_msgs.point_cloud2 as pc2
import rospy
import numpy as np
from time import sleep
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2
from pioneer_motion.motion import Motion

class Workspace:
    def __init__(self):
        rospy.init_node('pioneer_workspace', anonymous=False)
        rospy.loginfo("[PW] Pioneer Main Workspace- Running")
        
        self.motion       = Motion()
        self.scan_offset  = 40
        self.init_head_p  = 35
        self.lidar_finish = False
        # self.point_clouds = np.empty((0,3))
        self.point_clouds = []
        self.pcl_intens   = []

        ## Subscriber
        rospy.Subscriber('/robotis/sensor/assembled_scan', PointCloud2, self.point_cloud2_callback)
        rospy.Subscriber('/robotis/sensor/move_lidar',     String,      self.lidar_turn_callback)

    def wait_robot(self, msg):
        motion = self.motion
        while motion.status_msg != msg:
            pass # do nothing

    def point_cloud2_callback(self, data):
        pc          = ros_numpy.numpify(data)
        points      = np.zeros((pc.shape[0],3))
        points[:,0] = pc['x']
        points[:,1] = pc['y']
        points[:,2] = pc['z']

        intensities = pc['intensities']
        # print(pc.dtype.names) # ('x', 'y', 'z', 'intensities', 'index')

        self.point_clouds.append(points)
        self.pcl_intens.append(intensities)

    def lidar_turn_callback(self, msg):
        if msg.data == "end":
            rospy.loginfo("[PW] Scan finished")
            self.lidar_finish = True
            self.plot_point_cloud()

    def plot_point_cloud(self):
        sleep(8)
        pcl_data    = self.point_clouds[-1]
        intens_data = self.pcl_intens[-1]

        rospy.loginfo("[PW] Length point cloud : {}".format(pcl_data.shape))
        visual_ptk = pptk.viewer(pcl_data)
        visual_ptk.attributes(intens_data)

        # v.attributes(points[['r', 'g', 'b']] / 255., points['i'])
        # color      = pcl_data.copy()
        # color[:,:] = [255, 0, 0]
        # visual_ptk.attributes(color)
        # visual_ptk.set(point_size=0.001)

    def run(self):
        motion = self.motion
        motion.publisher_(motion.module_control_pub, "head_control_module", latch=True)

        motion.set_head_joint_states(['head_y', 'head_p'], [0, self.init_head_p])
        self.wait_robot("Head movement is finished.")
        rospy.loginfo('[PW] Head Init ...')

        # motion.publisher_(motion.move_lidar_pub, "start") # scan full head_p
        motion.publisher_(motion.move_lidar_range_pub, np.radians(self.scan_offset+1)) # scan head with range

        while not rospy.is_shutdown():
            pass
            # motion.publisher_(motion.move_lidar_pub, "start") # scan full head_p
            # motion.publisher_(motion.move_lidar_range_pub, np.radians(self.scan_offset+1)) # scan head with range
            # break


        motion.kill_threads()

if __name__ == '__main__':
    ws = Workspace()
    ws.run()