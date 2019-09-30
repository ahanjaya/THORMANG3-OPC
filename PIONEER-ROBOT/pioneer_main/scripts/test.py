#!/usr/bin/env python3

import os
import pcl
import pptk
import rospy
import rospkg
import ros_numpy
import numpy as np
from time import sleep
from std_msgs.msg import String
import matplotlib.pyplot as plt
from sensor_msgs.msg import PointCloud2
from pioneer_motion.motion import Motion

np.set_printoptions(suppress=True)

class Lidar_Cross_Arm:
    def __init__(self):
        rospy.init_node('pioneer_cross_arm_lidar', anonymous=False)
        rospy.loginfo("[CA] Pioneer Cross Arm Lidar- Running")

        rospack           = rospkg.RosPack()
        self.pcl_path     = rospack.get_path("pioneer_main") + "/data/cross_arm/"
        self.motion       = Motion()
        self.scan_offset  = 50
        self.init_head_p  = 30
        self.point_clouds = None
        self.lidar_finish = False
        self.first_edge   = False
        self.debug        = False

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

    def plots_(self, axes_, x_, y_, legend_=None, scatter=False, xlabel_="", ylabel_="", title_=""):
        if scatter:
            axes_.scatter(x_, y_, label=legend_)
        else:
            axes_.plot(x_, y_, 'o-', label=legend_)
        
        axes_.set_xlabel(xlabel_)
        axes_.set_ylabel(ylabel_)
        axes_.set_title(title_)
        # axes_.set_xlim([-0.3, 0.3])
        # axes_.set_xscale('linear')
        # axes_.invert_xaxis()
        axes_.grid()  

    def check_consecutive(self, arr):
        arr  = np.array(arr)
        ones = np.where(arr == 1)

        if ones[0].size != 0:
            l = list(ones[0])
            return l == list(range(min(l), max(l)+1))
        else:
            return False

    def middle_consecutive(self, arr):
        arr  = np.array(arr)
        ones = np.where(arr == 1)
        return (ones[0][0] + ones[0][-1]) // 2

    def filtering_raw_data(self, data):
        # plot original data
        # self.plot_point_cloud('point_clouds', data) # <-- plot

        # # Flip point cloud
        # flip_point_clouds = np.empty_like(data)
        # flip_point_clouds[:,0] = data[:,1]
        # flip_point_clouds[:,1] = data[:,0]
        # flip_point_clouds[:,2] = data[:,2]
        # flip_point_clouds[:,3] = data[:,3]
        # print(flip_point_clouds)
        # self.plot_point_cloud('flip point_clouds', flip_point_clouds) # <-- plot

        # Removing Table <-- this code will be removed on real practice
        data = data[ np.where( (data[:,1] >= 0.4) )]
        # self.plot_point_cloud('point_clouds', data) # <-- plot
        
        # Sorting point cloud
        data = data[np.argsort(data[:,1])] # Y axis
        # self.plot_point_cloud('point_clouds', data) # <-- plot

        # square scaning
        ymin = np.round( np.min(data[:,1]), 1) - 0.1
        ymax = np.round( np.max(data[:,1]), 1) + 0.1
        y_step_size = 0.025 #0.01
        y_step      = np.arange(ymin, ymax, y_step_size)

        # rospy.loginfo('[CA] X Point: {}'.format( (xmin, xmax) ))
        # rospy.loginfo('[CA] Y Point: {}'.format( (ymin, ymax) ))
        # print(y_step)
        # print()

        z_list = []
        z_prev = None

        for y in range(len(y_step)):
            y_ll = y_step[y]
            y_ul = y_step[y] + y_step_size
            # print('Y : {:.3f}, {:.3f}'.format(y_ll, y_ul) )

            layer = data[np.where( (data[:,1] >= y_ll) & (data[:,1] < y_ul) )]
            if layer.size != 0:
                zmax = np.max(layer[:,2])
                # print('\t\ttotal data: {}, max z : {:.4f}'.format(len(layer), zmax ) )

                z_list.append(zmax)
                if z_prev != None:
                    diff = zmax-z_prev
                    if diff <= -0.5:
                        y_lim = y_ll
                        break
                z_prev = zmax
            # print()

        if self.debug:
            _, axes2D = plt.subplots(nrows=1, ncols=1)
            self.plots_(axes2D, y_step[:len(z_list)], z_list, legend_=None, scatter=False, \
                xlabel_="y-layer", ylabel_="z-height", title_="Side View")
        
        human_body = data[np.where( (data[:,1] >= ymin) & (data[:,1] < y_lim) )]
        # self.plot_point_cloud('human_body', human_body) # <-- plot

        # Reference point
        x_ref     = ( np.min(human_body[:,0]) + np.max(human_body[:,0]) ) / 2
        y_ref     = np.min(human_body[:,1])
        z_ref     = np.max(human_body[:,2])
        ref_point = np.array([ [x_ref, y_ref, z_ref] ])
        # rospy.loginfo('[CA] Ref. Point: {}'.format(ref_point))

        # Euclidean Distance of 3D Point
        eucl_dist = np.linalg.norm(ref_point - human_body[:,:3], axis=1)

        # Filter euclidean distance
        human_body = human_body[ np.where( (eucl_dist <= 0.8) )] #0.8
        # self.plot_point_cloud('human_body', human_body, big_point=True, color=True )

        p   = pcl.PointCloud(np.array(human_body[:,:3], dtype=np.float32))
        sor = p.make_voxel_grid_filter()
        sor.set_leaf_size(0.01, 0.01, 0.01)
        filtered = sor.filter()

        filtered_human_body = np.asarray(filtered) 
        self.plot_point_cloud('filtered_human_body', filtered_human_body) #, big_point=True, color=True )

        # return human_body
        return filtered_human_body

    def paw_decision(self, human_body):
        # Reference point
        x_ref     = ( np.min(human_body[:,0]) + np.max(human_body[:,0]) ) / 2
        y_ref     = np.min(human_body[:,1])
        z_ref     = np.max(human_body[:,2])
        ref_point = np.array([ [x_ref, y_ref, z_ref] ])

        # 2D square scaning
        xmin = np.round( np.min(human_body[:,0]), 1) - 0.1
        xmax = np.round( np.max(human_body[:,0]), 1) + 0.1
        ymin = np.round( np.min(human_body[:,1]), 1) - 0.1
        ymax = np.round( np.max(human_body[:,1]), 1) + 0.1

        len_square = 0.025 #0.01
        x_step = np.arange(xmin, xmax, len_square)
        y_step = np.arange(ymin, ymax, len_square)

        edge_point = []
        for y in range(len(y_step)):
            y_ll   = y_step[y]
            y_ul   = y_step[y] + len_square 
            # print('Y : {:.2f}, {:.2f}'.format(y_ll, y_ul) )
            binary = []
            for x in range(len(x_step)):
                x_ll   = x_step[x]
                x_ul   = x_step[x] + len_square
                # print('\t\tX : {:.2f}, {:.2f}'.format(x_ll, x_ul) )
                small_cube = human_body[np.where( (human_body[:,0] >= x_ll) & (human_body[:,0] < x_ul) & \
                                                    (human_body[:,1] >= y_ll) & (human_body[:,1] < y_ul) )]
                if small_cube.size != 0:  binary.append(1)
                else:                     binary.append(0)
            
            # print('\t', binary)
            edge_point.append(binary)
            if self.check_consecutive(binary):
                if not all(elem == 0 for elem in edge_point[-2]):
                # print('\t\t',edge_point[-2])
                    ymax = y_ul
                    print(ymax)
                    break

        # show cross arm
        crossed_arm = human_body[np.where( (human_body[:,1] >= ymin) & (human_body[:,1] < ymax))]
        self.plot_point_cloud('crossed_arm', crossed_arm, big_point=True, color=True )
            
        # seperate cross arm by screen
        x_mid        = self.middle_consecutive(edge_point[-1])
        left_screen  = crossed_arm[np.where( (crossed_arm[:,0] <= x_step[x_mid] ) )]
        right_screen = crossed_arm[np.where( (crossed_arm[:,0] > x_step[x_mid] ) )]

        self.plot_point_cloud('left_screen', left_screen,  big_point=True, color=True )
        self.plot_point_cloud('right_screen', right_screen, big_point=True, color=True )

        # calculate average distance of each screen
        left_screen_dist  = np.linalg.norm(ref_point - left_screen[:,:3], axis=1)
        left_screen_dist  = np.mean(left_screen_dist)
        # left_screen_dist   = np.min(left_screen_dist)

        right_screen_dist = np.linalg.norm(ref_point - right_screen[:,:3], axis=1)
        right_screen_dist = np.mean(right_screen_dist)
        # right_screen_dist  = np.min(right_screen_dist)

        # decision
        self.final_decision(left_screen_dist, right_screen_dist)

        if self.debug:
            temp_y = []
            for y in range(len(y_step)):
                ll   = y_step[y]
                ul   = y_step[y] + len_square
                temp = human_body[ np.where( (human_body[:,1] >= ll) & (human_body[:,1] < ul))]
                temp_y.append(temp)
            
            _, axes2D = plt.subplots(nrows=1, ncols=1)
            for y in temp_y:
                self.plots_(axes2D, y[:,0], y[:,1], legend_=None, scatter=True, \
                    xlabel_="x-distance", ylabel_="y-distance", title_="2D Human Body")

            xlim_min, xlim_max = axes2D.get_xlim()
            ylim_min, ylim_max = axes2D.get_ylim()

            _, axes2D = plt.subplots(nrows=1, ncols=1)
            self.plots_(axes2D, crossed_arm[:,0], crossed_arm[:,1], legend_=None, scatter=True,\
                xlabel_="x-distance", ylabel_="y-distance", title_="2D Crossed Paw")
            axes2D.set_xlim([xlim_min, xlim_max])
            axes2D.set_ylim([ylim_min, ylim_max])

    def intersection_decision(self, human_body):
        # 2D square scaning
        xmin = np.round( np.min(human_body[:,0]), 1) - 0.1
        xmax = np.round( np.max(human_body[:,0]), 1) + 0.1
        ymin = np.round( np.min(human_body[:,1]), 1) - 0.1
        ymax = np.round( np.max(human_body[:,1]), 1) + 0.1

        len_square = 0.025 #0.01
        x_step = np.arange(xmin, xmax, len_square)
        y_step = np.arange(ymin, ymax, len_square)

        edge_point = []
        for y in range(len(y_step)):
            y_ll   = y_step[y]
            y_ul   = y_step[y] + len_square 
            # print('Y : {:.2f}, {:.2f}'.format(y_ll, y_ul) )
            binary = []
            for x in range(len(x_step)):
                x_ll   = x_step[x]
                x_ul   = x_step[x] + len_square
                # print('\t\tX : {:.2f}, {:.2f}'.format(x_ll, x_ul) )
                small_cube = human_body[np.where( (human_body[:,0] >= x_ll) & (human_body[:,0] < x_ul) & \
                                                    (human_body[:,1] >= y_ll) & (human_body[:,1] < y_ul) )]
                if small_cube.size != 0:  binary.append(1)
                else:                     binary.append(0)
            
            # print('\t', binary)
            edge_point.append(binary)
            if self.check_consecutive(binary):
                if not self.first_edge:
                    if not all(elem == 0 for elem in edge_point[-2]):
                        self.first_edge = True
            else:
                # check last intersection
                if self.first_edge:
                    ymin = y_ll
                    break

        # show cross arm
        crossed_arm = human_body[np.where( (human_body[:,1] >= ymin) & (human_body[:,1] < ymin + 0.07))]
        # self.plot_point_cloud('crossed_arm', crossed_arm, big_point=True, color=True )

        # filter noise
        x_ref     = ( np.min(crossed_arm[:,0]) + np.max(crossed_arm[:,0]) ) / 2
        y_ref     = np.min(crossed_arm[:,1])
        z_ref     = np.max(crossed_arm[:,2])
        ref_point = np.array([ [x_ref, y_ref, z_ref] ])

        # Filter euclidean distance
        eucl_dist          = np.linalg.norm(ref_point - crossed_arm[:,:3], axis=1)
        filter_crossed_arm = crossed_arm[ np.where( (eucl_dist <= 0.2) )]
        self.plot_point_cloud('filter_crossed_arm', filter_crossed_arm, big_point=True, color=True )
           
        # seperate cross arm by screen
        x_mid        = self.middle_consecutive(edge_point[-2])
        left_screen  = filter_crossed_arm[np.where( (filter_crossed_arm[:,0] <= x_step[x_mid] ) )]
        right_screen = filter_crossed_arm[np.where( (filter_crossed_arm[:,0] > x_step[x_mid] ) )]

        self.plot_point_cloud('left_screen', left_screen,  big_point=True, color=True )
        self.plot_point_cloud('right_screen', right_screen, big_point=True, color=True )

        # calculate average z height each screen
        left_screen_dist  = np.mean(left_screen[:,2])
        right_screen_dist = np.mean(right_screen[:,2])

        # decision
        self.final_decision(left_screen_dist, right_screen_dist)

        if self.debug:
            temp_y = []
            for y in range(len(y_step)):
                ll   = y_step[y]
                ul   = y_step[y] + len_square
                temp = human_body[ np.where( (human_body[:,1] >= ll) & (human_body[:,1] < ul))]
                temp_y.append(temp)
            
            _, axes2D = plt.subplots(nrows=1, ncols=1)
            for y in temp_y:
                self.plots_(axes2D, y[:,0], y[:,1], legend_=None, scatter=True, \
                    xlabel_="x-distance", ylabel_="y-distance", title_="2D Human Body")

            xlim_min, xlim_max = axes2D.get_xlim()
            ylim_min, ylim_max = axes2D.get_ylim()

            _, axes2D = plt.subplots(nrows=1, ncols=1)
            self.plots_(axes2D, filter_crossed_arm[:,0], filter_crossed_arm[:,1], legend_=None, scatter=True,\
                xlabel_="x-distance", ylabel_="y-distance", title_="2D Crossed Arm")
            axes2D.set_xlim([xlim_min, xlim_max])
            axes2D.set_ylim([ylim_min, ylim_max])

    def final_decision(self, left_screen_dist, right_screen_dist):
        rospy.loginfo('\n[CA] Left screen dist : {}'.format(left_screen_dist))
        rospy.loginfo('[CA] Right screen dist : {}'.format(right_screen_dist))
        
        if left_screen_dist < right_screen_dist:
            rospy.loginfo('[CA] Left arm on Top')
            rospy.loginfo('[CA] Right arm on Bottom')
        else:
            rospy.loginfo('[CA] Right arm on Top')
            rospy.loginfo('[CA] Left arm on Bottom')
           
    def run(self):
        motion   = self.motion
        load_pcl = True

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

            # self.plot_point_cloud('point_clouds', self.point_clouds)
        else:
            counter = 10
            data  = np.load(self.pcl_path + "thormang3_cross_arm_pcl-" + str(counter) + ".npz")
            self.point_clouds = data['pcl']
            # self.plot_point_cloud('point_clouds', self.point_clouds) # <-- plot

        # process data
        human_body = self.filtering_raw_data(self.point_clouds)
        # self.paw_decision(human_body)
        # self.intersection_decision(human_body)

        if self.debug:
            plt.show(block=False)
            input("Press [enter] to close.\n")
            plt.close('all')
        
        motion.kill_threads()

if __name__ == '__main__':
    lca = Lidar_Cross_Arm()
    lca.run()