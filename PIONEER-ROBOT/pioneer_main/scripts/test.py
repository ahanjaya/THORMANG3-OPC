#!/usr/bin/env python3
import json
import rospkg
import numpy as np
import matplotlib.pyplot as plt

rospack        = rospkg.RosPack()
# aruco_ws_path  = rospack.get_path("pioneer_main") + "/config/thormang3_aruco_ws.json"
aruco_ws_path  = rospack.get_path("pioneer_main") + "/config/thormang3_aruco_ws.yaml"
np.set_printoptions(suppress=True)

# def parse_config():
#     global y_frame_min, y_frame_max
#     global x_frame_min, x_frame_max
#     global ik_xmin, ik_xmax, ik_ysteps

#     with open(aruco_ws_path, 'r') as f:
#         config = json.load(f)

#     # y move inside
#     for ystep in reversed(ik_ysteps):
#         x = ik_xmax
#         y = np.round(ystep, 2)

#         x = str(x)
#         y = str(y)
#         data = config[ x + ',' + y]
#         y_frame_min.append(data[1])

#     # y move outside
#     for ystep in ik_ysteps:
#         x = ik_xmin
#         y = np.round(ystep, 2)

#         x = str(x)
#         y = str(y)
#         data = config[ x + ',' + y]
#         y_frame_max.append(data[1])

#     y_frame_min = int ( np.mean( np.array(y_frame_min) ) )
#     y_frame_max = int ( np.mean( np.array(y_frame_max) ) )

#     # x_move forward
#     for xstep in ik_xsteps:
#         x = np.round(xstep, 2)
#         y = ik_ymax

#         x = str(x)
#         y = str(y)
#         data = config[ x + ',' + y]
#         x_frame_min.append(data[0])

#     # x_move backward
#     for xstep in reversed(ik_xsteps):
#         x = np.round(xstep, 2)
#         y = ik_ymin

#         x = str(x)
#         y = str(y)
#         data = config[ x + ',' + y]
#         x_frame_max.append(data[0])




# # x_ik_min    = 0.15
# # x_ik_max    = 0.45
# # y_frame_min = 173
# # y_frame_max = 448

# global y_frame_min, y_frame_max
# y_frame_min = []
# y_frame_max = []
# x_frame_min = []
# x_frame_max = []

# s       = 20
# ik_xmin = 0.15
# ik_xmax = 0.45
# ik_xsteps = np.linspace(ik_xmin, ik_xmax, num=s)
# ik_ymin = 0.1
# ik_ymax = 0.24
# ik_ysteps = np.linspace(ik_ymin, ik_ymax, num=s//2)

# parse_config()

# print('y_frame_min: ', y_frame_min)
# print('y_frame_max: ', y_frame_max)

# x_frame_min_a = min(x_frame_min)
# x_frame_min_b = max(x_frame_min) 
# x_frame_max_a = min(x_frame_max)
# x_frame_max_b = max(x_frame_max)

# y_ws = range(y_frame_min, y_frame_max+1)


# while True:
#     y_frame_tar = int( input("\nY input: ") )

#     if y_frame_tar in y_ws:
#         x_ik = np.interp( y_frame_tar, [y_frame_min, y_frame_max], [ik_xmax, ik_xmin] )
#         print('x_ik :', x_ik)

#         x_frame_min = np.interp( y_frame_tar, [y_frame_min, y_frame_max], [x_frame_min_b, x_frame_min_a] )
#         x_frame_min = int(x_frame_min)
#         print('xframe min :', x_frame_min)
#         x_frame_max = np.interp( y_frame_tar, [y_frame_min, y_frame_max], [x_frame_max_b, x_frame_max_a] )
#         x_frame_max = int(x_frame_max)
#         print('xframe max :', x_frame_max)

#         x_ws = range(x_frame_min, x_frame_max+1)

#         x_frame_tar = int( input("x input: ") )
#         if x_frame_tar in x_ws:
#             y_ik = np.interp( x_frame_tar, [x_frame_min, x_frame_max], [ik_ymax, ik_ymin] )
#             print('y_ik :', y_ik)
#         else:
#             print('X target is out of robot range')
#     else:
#         print('Y target is out of robot range')




import yaml

B = dict(
    C = 'ca',
    D = 'hanj',
    E = 'ead')

C = dict(
    q = 'ca',
    e = 'h',
    t = 'e')


data = dict(
    P1 = 'a',
    p2 = B,
    p3 = C,
)

left = dict(data)

with open(aruco_ws_path, 'w') as outfile:
    yaml.dump(left, outfile, default_flow_style=False)
