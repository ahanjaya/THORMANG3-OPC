#!/usr/bin/env python3

import os
import pcl
import pptk
import math
import rospy
import rospkg
import ros_numpy
import numpy as np
from time import sleep
from std_msgs.msg import String
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from pioneer_motion.motion import Motion

np.set_printoptions(suppress=True)

class Cross_Arm:
    def __init__(self):
        rospy.init_node('pioneer_cross_arm', anonymous=False)
        rospy.loginfo("[CA] Pioneer Cross Arm- Running")

        rospack           = rospkg.RosPack()
        self.pcl_path     = rospack.get_path("pioneer_main") + "/data/cross_arm/"
        self.motion       = Motion()
        self.scan_offset  = 50
        self.init_head_p  = 30
        self.point_clouds = None
        self.lidar_finish = False
        self.debug        = True
        self.first_scan   = False

        ## Subscriber
        rospy.Subscriber('/robotis/sensor/assembled_scan', PointCloud2, self.point_cloud2_callback)
        rospy.Subscriber('/robotis/sensor/move_lidar',     String,      self.lidar_turn_callback)

    def wait_robot(self, msg):
        motion = self.motion
        while motion.status_msg != msg:
            pass # do nothing

    def point_cloud2_callback(self, data):
        pc          = ros_numpy.numpify(data)
        points      = np.zeros((pc.shape[0],4))
        points[:,0] = pc['x']
        points[:,1] = pc['y']
        points[:,2] = pc['z']
        points[:,3] = pc['intensities']
        # print(pc.dtype.names) # ('x', 'y', 'z', 'intensities', 'index')

        # print(len(points))
        self.point_clouds = points

    def lidar_turn_callback(self, msg):
        if msg.data == "end":
            rospy.loginfo("[CA] Scan finished")
            self.lidar_finish = True

    def plot_point_cloud(self, label, pcl_data, big_point=False, color=True):
        rospy.loginfo("[CA] {} - length pcl : {}".format(label, pcl_data.shape))
        display = True

        if display:
            visual_ptk = pptk.viewer(pcl_data[:,:3])
            if color:
                visual_ptk.attributes(pcl_data[:,-1])
            if big_point:
                visual_ptk.set(point_size=0.0025)
            
            # v.attributes(points[['r', 'g', 'b']] / 255., points['i'])
            # color      = pcl_data.copy()
            # color[:,:] = [255, 0, 0]
            # visual_ptk.attributes(color)

    def plots_(self, axes_, x_, y_, legend_=None, xlabel_="", ylabel_="", title_=""):
        axes_.plot(x_, y_, 'o-', label=legend_)
        axes_.set_xlabel(xlabel_)
        axes_.set_ylabel(ylabel_)
        axes_.set_title(title_)
        # axes_.set_xlim([-0.2, 0.2])
        # axes_.set_xscale('linear')
        axes_.invert_xaxis()
        axes_.grid()  

    def check_consecutive(self, arr):
        arr  = np.array(arr)
        ones = np.where(arr == 1)

        if ones[0].size != 0:
            l = list(ones[0])
            return l == list(range(min(l), max(l)+1))
        else:
            return False

    def group_consecutive(self, data, stepsize=1):
        group = np.split(data, np.where(np.diff(data) != stepsize)[0]+1)
        if len(group) == 2:
            left  = group[0][-1]
            right = group[1][0]
            mid   = (left + right) / 2
            return int(mid)
        else:
            return len(data)//2

    def run(self):
        motion   = self.motion
        load_pcl = False

        if not load_pcl:
            motion.publisher_(motion.module_control_pub, "head_control_module", latch=True)
            motion.set_head_joint_states(['head_y', 'head_p'], [0, self.init_head_p])
            self.wait_robot("Head movement is finished.")
            rospy.loginfo('[CA] Head Init ...')

            torque_lvl = 0
            motion.set_joint_torque(['torso_y', "l_arm_sh_r", "r_arm_sh_r"], torque_lvl)
            rospy.loginfo('[CA] Set Torque : {}%'.format(torque_lvl))

            motion.publisher_(motion.move_lidar_pub, "start") # scan full head_p
            # motion.publisher_(motion.move_lidar_range_pub, np.radians(self.scan_offset+1)) # scan head with range

            counter = len(os.walk(self.pcl_path).__next__()[2])
            while not self.lidar_finish:
                pass

            self.lidar_finish = False
            sleep(1)
            np.savez(self.pcl_path + "thormang3_cross_arm_pcl-" + str(counter) + ".npz", pcl=self.point_clouds)
            rospy.loginfo('[CA] save: thormang3_cross_arm_pcl-{}.npz'.format(counter))

            self.plot_point_cloud('point_clouds', self.point_clouds)
        else:
            counter = 6
            data  = np.load(self.pcl_path + "thormang3_cross_arm_pcl-" + str(counter) + ".npz")
            self.point_clouds = data['pcl']
            self.plot_point_cloud('point_clouds', self.point_clouds) # <-- plot

        while not rospy.is_shutdown():
            # # Filtering point clouds area
            # area_conds    = np.where( (self.point_clouds[:,1] > -0.3) & (self.point_clouds[:,1] < 0.45) )
            # filter_area   = self.point_clouds[area_conds]
            # # self.plot_point_cloud('filter_area', filter_area) # <-- plot

            # # Reference point
            # x_ref = np.min(filter_area[:,0])
            # z_ref = np.max(filter_area[:,2])
            # y_ref = ( np.min(filter_area[:,1]) + np.max(filter_area[:,1]) ) / 2
            # ref_point = np.array([ [x_ref, y_ref, z_ref , 0] ])
            # rospy.loginfo('[CA] Ref. Point: {}'.format(ref_point))

            # # # Append reference point to filter_area
            # # filter_area = np.append (filter_area, ref_point, axis = 0)
            # # self.plot_point_cloud('filter_area', filter_area, big_point=True, color=True )

            # # Euclidean Distance of 3D Point
            # eucl_dist = np.linalg.norm(ref_point[:,:3] - filter_area[:,:3], axis=1)

            # # Filter euclidean distance
            # human_body = filter_area[ np.where( (eucl_dist <= 0.8) )] #0.8
            # # human_body = filter_area[ np.where(  (eucl_dist >= 0.7) & (eucl_dist <= 0.8))] #0.8
            # human_body = human_body[np.argsort(human_body[:,0])]            # sorting point cloud
            # human_body = human_body[np.where( (human_body[:,1] <= 0.3) )]   # removing noise
            # self.plot_point_cloud('human_body', human_body, big_point=True, color=True )

            # # 2D square scaning
            # xmin = np.min(human_body[:,0])
            # xmax = np.max(human_body[:,0])
            # ymin = np.min(human_body[:,1])
            # ymax = np.max(human_body[:,1])
            # # rospy.loginfo('[CA] X Point: {}'.format( (xmin, xmax) ))
            # # rospy.loginfo('[CA] Y Point: {}'.format( (ymin, ymax) ))
            # # print()

            # len_square = 0.025 #0.01 #0.025
            # xmin = np.round( np.min(human_body[:,0]), 1) - 0.1
            # xmax = np.round( np.max(human_body[:,0]), 1) + 0.1
            # ymin = np.round( np.min(human_body[:,1]), 1) - 0.1
            # ymax = np.round( np.max(human_body[:,1]), 1) + 0.1
            # # rospy.loginfo('[CA] X Point: {}'.format( (xmin, xmax) ))
            # # rospy.loginfo('[CA] Y Point: {}'.format( (ymin, ymax) ))

            # x_step = np.arange(xmin, xmax, len_square)
            # y_step = np.arange(ymin, ymax, len_square)
            # # print(x_step)
            # # print(y_step)
            # # print()

            # edge_point = []

            # for x in range(len(x_step)):
            #     x_ll   = x_step[x]
            #     x_ul   = x_step[x] + len_square 
            #     # print('X : {:.2f}, {:.2f}'.format(x_ll, x_ul) )
            #     binary = []
            #     for y in range(len(y_step)):
            #         y_ll   = y_step[y]
            #         y_ul   = y_step[y] + len_square
            #         # print('\t\tY : {:.2f}, {:.2f}'.format(y_ll, y_ul) )

            #         small_cube = human_body[np.where( (human_body[:,0] >= x_ll) & (human_body[:,0] < x_ul) & \
            #                                             (human_body[:,1] >= y_ll) & (human_body[:,1] < y_ul) )]

            #         if small_cube.size != 0:  binary.append(1)
            #         else:                     binary.append(0)
                
            #     # print('\t', binary)
            #     edge_point.append(binary)
            #     if self.check_consecutive(binary):
            #         # print('\tfill')
            #         idx_edge_point = len(edge_point) - 2
            #         xmax = x_ul
            #         break

            #         # if not self.first_scan:
            #         #     idx_edge_point = len(edge_point) - 2
            #         #     xmin = x_ul
            #         #     self.first_scan = True
            #     # else:
            #     #     if self.first_scan:
            #     #         xmax = x_ll
            #     #         break

            # # crossed_arm = human_body[np.where( (human_body[:,0] >= xmin) & (human_body[:,0] < xmin+0.08))]

            # # show cross arm
            # crossed_arm = human_body[np.where( (human_body[:,0] >= xmin) & (human_body[:,0] < xmax))]
            # self.plot_point_cloud('crossed_arm', crossed_arm, big_point=True, color=True )
            
            # # seperate cross arm by screen
            # last_cross   = np.array(edge_point[idx_edge_point])
            # last_cross   = np.where(last_cross == 1)
            # y_mid        = self.group_consecutive(last_cross[0])
            # left_screen  = crossed_arm[np.where( (crossed_arm[:,1] > y_step[y_mid] ) )]
            # right_screen = crossed_arm[np.where( (crossed_arm[:,1] <= y_step[y_mid] ) )]

            # self.plot_point_cloud('left_screen', left_screen,  big_point=True, color=True )
            # self.plot_point_cloud('right_screen', right_screen, big_point=True, color=True )

            # # calculate average distance of each screen
            # left_screen_dist  = np.linalg.norm(ref_point[:,:3] - left_screen[:,:3], axis=1)
            # left_screen_dist  = np.mean(left_screen_dist)
            # # left_screen_dist   = np.min(left_screen_dist)

            # right_screen_dist = np.linalg.norm(ref_point[:,:3] - right_screen[:,:3], axis=1)
            # right_screen_dist = np.mean(right_screen_dist)
            # # right_screen_dist  = np.min(right_screen_dist)

            # # conclusion
            # print()
            # rospy.loginfo('[CA] Left screen dist : {}'.format(left_screen_dist))
            # rospy.loginfo('[CA] Right screen dist : {}'.format(right_screen_dist))
            
            # if left_screen_dist < right_screen_dist:
            # # if right_screen_dist < left_screen_dist:
            #     rospy.loginfo('[CA] Left Arm on Top')
            # else:
            #     rospy.loginfo('[CA] Right Arm on Top')

            # if self.debug:
            #     # sort_x
            #     temp_x = []
            #     for i in range(len(x_step)):
            #         ll   = x_step[i]
            #         ul   = x_step[i] + 0.1
            #         temp = human_body[ np.where( (human_body[:,0] >= ll) & (human_body[:,0] < ul))]
            #         temp_x.append(temp)
                
            #     _, axes2D = plt.subplots(nrows=1, ncols=1)
            #     # fig3d_1   = plt.figure()
            #     # axes3D    = fig3d_1.add_subplot(111, projection='3d')
            #     for i in temp_x:
            #         axes2D.scatter(i[:,1], i[:,0])
            #         # axes3D.scatter(i[:,1], i[:,0], i[:,2], marker='.')

            #         # eucl_dist = np.linalg.norm(ref_point[:,:3] - i[:,:3], axis=1)
            #         # axes3D.scatter(i[:,1], i[:,0], eucl_dist, marker='.')

            #     axes2D.invert_xaxis()
            #     # axes3D.invert_xaxis()
                
            #     plt.show(block=False)
            #     input("Press [enter] to close.\n")
            #     plt.close('all')

            break

        motion.kill_threads()

if __name__ == '__main__':
    ca = Cross_Arm()
    ca.run()