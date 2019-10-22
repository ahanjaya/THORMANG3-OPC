#!/usr/bin/env python3

import os
import pcl
import glob
import pptk
import rospy
import rospkg
import ros_numpy
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from pyntcloud import PyntCloud
from mpl_toolkits.mplot3d import Axes3D

class Preprocess_Data:
    def __init__(self):
        rospy.init_node('pioneer_cross_preprocess_data')
        rospy.loginfo("[CA] Pioneer Collect Data Cross Arm- Running")

        rospack           = rospkg.RosPack()
        self.pcl_dataset  = rospack.get_path("pioneer_main") + "/data/cross_arm/cross_arm_dataset.npz"
        self.pcl_raw_path = rospack.get_path("pioneer_main") + "/data/cross_arm/raw_pcl/"
        self.pcl_cam_path = rospack.get_path("pioneer_main") + "/data/cross_arm/cam/"
        self.main_rate    = rospy.Rate(1)
        self.point_clouds = None
        self.visual_ptk1  = None
        self.visual_ptk2  = None
        self.show_plt     = False
        self.debug        = False
        self.data         = []
        self.labels       = []

        # labeling data
        self.arm = 'left_arm_top'
        # self.arm = 'right_arm_top'

    def plot_point_cloud(self, label, pcl_data, big_point=False, color=True):
        rospy.loginfo("[Pre.] {} - length pcl : {}".format(label, pcl_data.shape))
        visual_ptk = pptk.viewer(pcl_data[:,:3])

        if color:
            visual_ptk.attributes(pcl_data[:,-1])
        if big_point:
            visual_ptk.set(point_size=0.0025)

        return visual_ptk

    def plots_(self, axes_, x_, y_, legend_=None, xlabel_="", ylabel_="", title_=""):
        axes_.plot(x_, y_, 'o-', label=legend_)
        axes_.set_xlabel(xlabel_)
        axes_.set_ylabel(ylabel_)
        axes_.set_title(title_)
        axes_.grid()

    def filter_raw_data(self, data):
        # Sorting point cloud
        data   = data[np.argsort(data[:,0])] # X axis
        data_x = data[:,0]

        # square scaning
        x_min = np.round( np.min(data_x), 1) - 0.1
        x_max = np.round( np.max(data_x), 1) + 0.1
        x_step_size = 0.025 #0.01
        y_step      = np.arange(x_min, x_max, x_step_size)
        z_list      = []
        z_prev      = None

        for y in range(len(y_step)):
            y_ll  = y_step[y]
            y_ul  = y_step[y] + x_step_size
            layer = data[np.where( (data_x >= y_ll) & (data_x < y_ul) )]

            if layer.size != 0:
                zmax = np.max(layer[:,2])
                z_list.append(zmax)
                if z_prev != None:
                    diff = zmax-z_prev
                    if diff <= -0.5:
                        y_lim = y_ll
                        break
                z_prev = zmax

        if self.show_plt:
            _, axes2D = plt.subplots(nrows=1, ncols=1)
            self.plots_(axes2D, y_step[:len(z_list)], z_list, legend_=None,\
                xlabel_="y-layer", ylabel_="z-height", title_="Filtering Human Body on Point Cloud")
        
        # first filter, by z layer
        human_body = data[np.where( (data_x >= x_min) & (data_x < y_lim) )]

        # second filter by euclidean distance
        x_ref      = np.min(human_body[:,0])
        y_ref      = (np.min(human_body[:,1]) + np.max(human_body[:,1])) / 2
        z_ref      = np.max(human_body[:,2])
        ref_point  = np.array([ [x_ref, y_ref, z_ref] ])
        # rospy.loginfo('[CAL] Ref. Point: {}'.format(ref_point))

        eucl_dist  = np.linalg.norm(ref_point - human_body[:,:3], axis=1)
        human_body = human_body[ np.where( (eucl_dist <= 0.8) )] #0.8
        # return human_body

        pcl_       = pcl.PointCloud(np.array(human_body[:,:3], dtype=np.float32))
        sor        = pcl_.make_voxel_grid_filter()
        # sor.set_leaf_size(0.01, 0.01, 0.01)
        sor.set_leaf_size(0.02, 0.02, 0.02)
        filtered   = sor.filter()
        human_body = np.asarray(filtered) 
        return human_body

    def voxelization(self, human_body):
        dataset      = pd.DataFrame({'x': human_body[:,0], 'y': human_body[:,1], 'z': human_body[:,2]})
        cloud        = PyntCloud(dataset)
        voxelgrid_id = cloud.add_structure("voxelgrid", n_x=32, n_y=32, n_z=32)
        voxelgrid    = cloud.structures[voxelgrid_id]
        x_cords      = voxelgrid.voxel_x
        y_cords      = voxelgrid.voxel_y
        z_cords      = voxelgrid.voxel_z
        voxel        = np.zeros((32, 32, 32)).astype(np.bool)

        for x, y, z in zip(x_cords, y_cords, z_cords):
            voxel[x][y][z] = True
        return voxel

    def close_all(self):
        if self.visual_ptk1 is not None:
            self.visual_ptk1.close()

        if self.visual_ptk2 is not None:                
            self.visual_ptk2.close()
        
        plt.close('all')

    def run(self):
        raw_files = os.listdir(self.pcl_raw_path)
        # raw_files = natsort.natsorted(raw_files)
        rospy.loginfo('[Pre.] Raw data : {}'.format(raw_files))
        print()

        for f in raw_files:
            file_name         = self.pcl_raw_path + f
            pcl_file          = np.load(file_name)
            self.point_clouds = pcl_file['pcl']
            # self.visual_ptk1  = self.plot_point_cloud(f, self.point_clouds) # <-- plot

            try:
                # filter human body
                human_body        = self.filter_raw_data(self.point_clouds)
                self.visual_ptk2  = self.plot_point_cloud(f, human_body, big_point=True, color=True )

                # voxelization
                voxel             = self.voxelization(human_body)
                # print(voxel)

                if 'left_arm_top' in f:
                    y_label = 0
                elif 'right_arm_top' in f:
                    y_label = 1
                rospy.loginfo('[Pre.] Y label : {}'.format(y_label))

                # acquiring data
                self.data.append(voxel)
                self.labels.append(y_label)

                if self.show_plt:
                    fig = plt.figure()
                    ax = fig.gca(projection='3d')
                    ax.voxels(voxel)
                    plt.show(block=False)
            except:
                pass

            self.main_rate.sleep()

            if self.debug:
                state = input("\nContinue : ")
                if state == 'q':
                    rospy.loginfo('[Pre.] Exit..')
                    self.close_all()
                    break
                else:
                    self.close_all()
            else:
                self.close_all()

        self.data   = np.array(self.data)
        self.labels = np.array(self.labels)
        rospy.loginfo('[Pre.] Total data : {}'.format(self.data.shape))
        rospy.loginfo('[Pre.] Total label: {}'.format(self.labels.shape))
        np.savez(self.pcl_dataset, data=self.data, labels=self.labels)

        rospy.loginfo('[Pre.] Exit Loop')

if __name__ == '__main__':
    collect = Preprocess_Data()
    collect.run()