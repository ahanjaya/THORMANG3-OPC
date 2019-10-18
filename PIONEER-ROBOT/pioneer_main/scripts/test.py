#!/usr/bin/env python3

import rospy
import rospkg
import numpy as np
import random
import pptk
from pyntcloud import PyntCloud

# import numpy as np
# import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D

# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')

# # your real data here - some 3d boolean array
# x, y, z = np.indices((10, 10, 10))
# print(x,y,z)
# voxels = (x == y) | (y == z)
# ax.voxels(voxels)
# plt.show()

rospy.init_node('pioneer_', disable_signals=True)

def plot_point_cloud(label, pcl_data, big_point=False, color=True):
    rospy.loginfo("[CAL] {} - length pcl : {}".format(label, pcl_data.shape))
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

# rospack  = rospkg.RosPack()
# pcl_path = rospack.get_path("pioneer_main") + "/data/cross_arm/"

# counter = 1
# data = np.load(pcl_path + "thormang3_cross_arm_pcl-" + str(counter) + ".npz")
# point_clouds = data['pcl']

# print(point_clouds)
# # plot_point_cloud('point_clouds', point_clouds)

# voxelgrid_id = cloud.add_structure("voxelgrid", n_x=32, n_y=32, n_z=32)
# voxelgrid = cloud.structures[voxelgrid_id]
# # voxelgrid.plot(d=3, mode="density", cmap="hsv")

# x_cords = voxelgrid.voxel_x
# y_cords = voxelgrid.voxel_y
# z_cords = voxelgrid.voxel_z

import numpy as np
import pandas as pd
from pyntcloud import PyntCloud

points = pd.DataFrame(np.random.rand(1000, 3))
cloud = PyntCloud(points)