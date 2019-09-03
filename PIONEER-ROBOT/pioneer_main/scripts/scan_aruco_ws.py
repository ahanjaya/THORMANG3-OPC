#!/usr/bin/env python3

# import json
import yaml
import rospy
import rospkg
import numpy as np
from time import sleep
from geometry_msgs.msg import Point32
from pioneer_kinematics.kinematics import Kinematics

rospack        = rospkg.RosPack()
aruco_ws_path  = rospack.get_path("pioneer_main") + "/config/thormang3_aruco_ws.yaml"
np.set_printoptions(suppress=True)

def left_aruco_pos_callback(msg):
    global lmarker_x, lmarker_y
    lmarker_x = msg.x
    lmarker_y = msg.y

def right_aruco_pos_callback(msg):
    global rmarker_x, rmarker_y
    rmarker_x = msg.x
    rmarker_y = msg.y

def left_arm_pos_callback(msg):
    global left_tar_x, left_tar_y
    left_tar_x = msg.x
    left_tar_y = msg.y

def right_arm_pos_callback(msg):
    global right_tar_x, right_tar_y
    right_tar_x = msg.x
    right_tar_y = msg.y

def wait_robot(obj, msg):
    while obj.status_msg != msg:
        pass # do nothing

def logging(obj, arm, x, y, idx, config):
    global lmarker_x, lmarker_y, rmarker_x, rmarker_y
    # buffering
    sleep(1)
    wait_robot(obj, "End " + arm + " Trajectory")
    sleep(1)

    # wait until aruco stable
    if arm == 'Left Arm':
        while lmarker_x == -1 or lmarker_y == -1:
            pass
        config['P'+str(idx)] = dict(ik_x = x, ik_y = y, cx = lmarker_x, cy = lmarker_y)

    elif arm == 'Right Arm':
        while rmarker_x == -1 or rmarker_y == -1:
            pass
        config['P'+str(idx)] = dict(ik_x = x, ik_y = y, cx = rmarker_x, cy = rmarker_y)
    
    print('{0} (Points: {1})  IK X: {2} \t IK_Y: {3} \t CX:{4} \t CY:{5}'.format(arm, idx, x, y, lmarker_x, lmarker_y))

def load_config(arm):
    try:
        with open(aruco_ws_path, 'r') as f:
            aruco_ws = yaml.safe_load(f)
        return aruco_ws[arm]

    except yaml.YAMLError as exc:
        print(exc)
        return None

def parse_Yframe(data):
    y_frame_min = np.mean([ data['P2']['cy'], data['P3']['cy'] ], dtype=int)
    y_frame_max = np.mean([ data['P1']['cy'], data['P4']['cy'] ], dtype=int)

    ik_xmin = data['P1']['ik_x']
    ik_xmax = data['P2']['ik_x']

    return ( y_frame_min, y_frame_max, ik_xmin, ik_xmax )

def parse_Xframe(data):
    x_frame_min = ( data['P1']['cx'], data['P2']['cx'] )
    x_frame_max = ( data['P3']['cx'], data['P4']['cx'] )

    ik_ymin_lower = data['P4']['ik_y'] #0.1
    ik_ymax_lower = data['P1']['ik_y'] #0.24
    ik_ymin_upper = data['P3']['ik_y'] #0.05
    ik_ymax_upper = data['P2']['ik_y'] #0.34

    return ( x_frame_min, x_frame_max, ik_ymax_lower, ik_ymax_upper, ik_ymin_lower, ik_ymin_upper)

def main():
    rospy.init_node('pioneer_main', anonymous=False)
    rospy.loginfo("Pioneer Aruco Workspace - Running")

    left_aruco_pos_sub  = rospy.Subscriber("/pioneer/aruco/left_position",  Point32, left_aruco_pos_callback)
    right_aruco_pos_sub = rospy.Subscriber("/pioneer/aruco/right_position", Point32, right_aruco_pos_callback)
    left_arm_pos_sub    = rospy.Subscriber("/pioneer/aruco/left_arm_point", Point32, left_arm_pos_callback)
    right_arm_pos_sub   = rospy.Subscriber("/pioneer/aruco/right_arm_point", Point32, right_arm_pos_callback)

    # global variables
    global lmarker_x, lmarker_y, rmarker_x, rmarker_y
    global left_tar_x, left_tar_y, right_tar_x, right_tar_y
    
    scan_workspace = False
    save_config    = False
    left_ws        = {}
    right_ws       = {}
    lik_xmin       = None
    lik_xmax       = None

    ##################
    ## Kinematics
    ##################
    kinematics = Kinematics()
    kinematics.publisher_(kinematics.module_control_pub, "manipulation_module", latch=True)  # <-- Enable Manipulation mode
    kinematics.publisher_(kinematics.send_ini_pose_msg_pub, "ini_pose", latch=True)
    wait_robot(kinematics, "End Init Trajectory")

    # set init head & torso
    kinematics.set_joint_pos(['head_p', 'head_y', 'torso_y'], [30, 0, 0])
    # Gripper
    kinematics.set_gripper("left_arm", 0, 0)
    kinematics.set_gripper("right_arm", 0, 0)
    kinematics.set_joint_pos(['l_arm_finger45_p', 'r_arm_finger45_p'], [180, 180])
    sleep(2)
    rospy.loginfo('Finish Init Head & Gripper')

    lik_ymin_lower = 0.1
    lik_ymax_lower = 0.24
    lik_ymin_upper = 0.05
    lik_ymax_upper = 0.34
    lik_xmin       = 0.15
    lik_xmax       = 0.45

    lp1 = (lik_xmin, lik_ymax_lower)
    lp2 = (lik_xmax, lik_ymax_upper)
    lp3 = (lik_xmax, lik_ymin_upper)
    lp4 = (lik_xmin, lik_ymin_lower)
    left_arm_pts = [ lp1, lp2, lp3, lp4 ]

    rik_ymin_lower = -0.1
    rik_ymax_lower = -0.24
    rik_ymin_upper = -0.05
    rik_ymax_upper = -0.34
    rik_xmin       = 0.15
    rik_xmax       = 0.45

    rp1 = (rik_xmin, rik_ymax_lower)
    rp2 = (rik_xmax, rik_ymax_upper)
    rp3 = (rik_xmax, rik_ymin_upper)
    rp4 = (rik_xmin, rik_ymin_lower)
    right_arm_pts = [ rp1, rp2, rp3, rp4 ]

    if scan_workspace:
        # Left Arm
        for idx, pts in enumerate(left_arm_pts):
            x, y = pts
            kinematics.set_kinematics_pose("left_arm" , 3.0, **{ 'x': x, 'y': y, 'z': 0.65, 'roll': -90.00, 'pitch': 0.00, 'yaw': 20.00 })
            logging(kinematics, 'Left Arm', x, y, idx+1, left_ws)

        # Right Arm
        for idx, pts in enumerate(right_arm_pts):
            x, y = pts
            kinematics.set_kinematics_pose("right_arm" , 3.0, **{ 'x': x, 'y': y, 'z': 0.65, 'roll': 90.00, 'pitch': 0.00, 'yaw': -20.00 })
            logging(kinematics, 'Right Arm', x, y, idx+1, right_ws)

        if save_config:
            aruco_ws = {}
            aruco_ws['left_arm']  = left_ws
            aruco_ws['right_arm'] = right_ws
            with open(aruco_ws_path, 'w') as f:
                yaml.dump(aruco_ws, f, default_flow_style=False)
    else:
        left_ws  = load_config('left_arm')
        right_ws = load_config('right_arm')
        rospy.loginfo('Loaded left_ws')
        rospy.loginfo('Loaded Right_ws')

    if left_ws != None:
        yleft_data    = parse_Yframe(left_ws)
        ly_frame_min  = yleft_data[0]
        ly_frame_max  = yleft_data[1]
        ly_ws         = range(ly_frame_min, ly_frame_max+1)
        lik_xmin      = yleft_data[2]
        lik_xmax      = yleft_data[3]

        xleft_data    = parse_Xframe(left_ws)
        lx_min_b      = xleft_data[0][0]
        lx_min_a      = xleft_data[0][1]
        lx_max_a      = xleft_data[1][0]
        lx_max_b      = xleft_data[1][1]

        lik_ymax_lower = xleft_data[2] #0.24
        lik_ymax_upper = xleft_data[3] #0.34
        lik_ymin_lower = xleft_data[4] #0.1
        lik_ymin_upper = xleft_data[5] #0.05

    if right_ws != None:
        yright_data   = parse_Yframe(right_ws)
        ry_frame_min  = yright_data[0]
        ry_frame_max  = yright_data[1]
        ry_ws         = range(ry_frame_min, ry_frame_max+1)
        rik_xmin      = yright_data[2]
        rik_xmax      = yright_data[3]

        xright_data   = parse_Xframe(right_ws)
        rx_max_b      = xright_data[0][0]
        rx_max_a      = xright_data[0][1]
        rx_min_a      = xright_data[1][0]
        rx_min_b      = xright_data[1][1]

        rik_ymax_lower = xright_data[2] #0.24
        rik_ymax_upper = xright_data[3] #0.34
        rik_ymin_lower = xright_data[4] #0.1
        rik_ymin_upper = xright_data[5] #0.05

    main_rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        main_rate.sleep()

        clicked = input("\nhave you click new point? ")
        if clicked == 'y':
            # Input by Mouse
            lx_tar = left_tar_x
            ly_tar = left_tar_y
            rx_tar = right_tar_x
            ry_tar = right_tar_y

            prev_lx = left_tar_x
            prev_ly = left_tar_y
            prev_rx = right_tar_x
            prev_ry = right_tar_y

            # print('rx_tar: {}, ry_tar: {} '.format(rx_tar, ry_tar))
            # print('lx_tar: {}, ly_tar: {} '.format(lx_tar, ly_tar))

        elif clicked == 'p':
            rx_tar = right_tar_x
            ry_tar = right_tar_y

            diff_rx = abs(rx_tar - prev_rx)
            diff_ry = abs(ry_tar - prev_ry)

            if rx_tar <= prev_rx:
                diff_rx = diff_rx * -1
            if ry_tar <= prev_ry:
                diff_ry = diff_ry * -1

            lx_tar  = prev_lx + diff_rx
            ly_tar  = prev_ly + diff_ry
            prev_lx = lx_tar
            prev_ly = ly_tar
            prev_rx = right_tar_x
            prev_ry = right_tar_y

            # print('rx_tar: {}, ry_tar: {}'.format(rx_tar, ry_tar))
            # print('diff_rx: {}, diff_ry: {}'.format(diff_rx, diff_ry))
            # print('lx_tar: {}, ly_tar: {}'.format(lx_tar, ly_tar))

        else:
            rospy.loginfo('Exit')
            break

        if ly_tar in ly_ws:
            # Left IK X Target
            lx_ik = np.interp( ly_tar, [ly_frame_min, ly_frame_max], [lik_xmax, lik_xmin] )

            # Mapping CX Frame
            lx_frame_min = np.interp( ly_tar, [ly_frame_min, ly_frame_max], [lx_min_a, lx_min_b] )
            lx_frame_min = int( np.round(lx_frame_min, 0) )
            lx_frame_max = np.interp( ly_tar, [ly_frame_min, ly_frame_max], [lx_max_a, lx_max_b] )
            lx_frame_max = int( np.round(lx_frame_max, 0) )

            # Mapping IK_Y
            lik_ymax     = np.interp( ly_tar, [ly_frame_min, ly_frame_max], [lik_ymax_upper, lik_ymax_lower] )
            lik_ymax     = np.round(lik_ymax, 4)
            lik_ymin     = np.interp( ly_tar, [ly_frame_min, ly_frame_max], [lik_ymin_upper, lik_ymin_lower] )
            lik_ymin     = np.round(lik_ymin, 4)

            lx_ws = range(lx_frame_min, lx_frame_max+1)
            if lx_tar in lx_ws:
                # Left IK Y Target
                ly_ik = np.interp( lx_tar, [lx_frame_min, lx_frame_max], [lik_ymax, lik_ymin] )

                rospy.loginfo('[Left Arm] Input Coor X: {0}, Y: {1}'.format(lx_tar, ly_tar))
                rospy.loginfo('[Left Arm] X_IK: {0:.2f}, Y_IK: {1:.2f}'.format(lx_ik, ly_ik))
                # kinematics.set_kinematics_pose("left_arm" , 2.0, **{ 'x': lx_ik, 'y': ly_ik, 'z': 0.65, 'roll': -90.00, 'pitch': 0.00, 'yaw': 20.00 })
                kinematics.set_kinematics_pose("left_arm" , 2.0, **{ 'x': lx_ik, 'y': ly_ik, 'z': 0.65, 'roll': -170.00, 'pitch': 20.00, 'yaw': -50.00 })
            else:
                rospy.logerr('[WS] X Frame target is out of range')
        else:
            rospy.logerr('[WS] Y Frame target is out of range')

        if ry_tar in ry_ws:
            # Left IK X Target
            rx_ik = np.interp( ry_tar, [ry_frame_min, ry_frame_max], [rik_xmax, rik_xmin] )

            # Mapping CX Frame
            rx_frame_min = np.interp( ry_tar, [ry_frame_min, ry_frame_max], [rx_min_a, rx_min_b] )
            rx_frame_min = int( np.round(rx_frame_min, 0) )
            rx_frame_max = np.interp( ry_tar, [ry_frame_min, ry_frame_max], [rx_max_a, rx_max_b] )
            rx_frame_max = int( np.round(rx_frame_max, 0) )

            # Mapping IK_Y
            rik_ymax     = np.interp( ry_tar, [ry_frame_min, ry_frame_max], [rik_ymax_upper, rik_ymax_lower] )
            rik_ymax     = np.round(rik_ymax, 4)
            rik_ymin     = np.interp( ry_tar, [ry_frame_min, ry_frame_max], [rik_ymin_upper, rik_ymin_lower] )
            rik_ymin     = np.round(rik_ymin, 4)

            rx_ws = range(rx_frame_min, rx_frame_max+1)
            if rx_tar in rx_ws:
                # Left IK Y Target
                ry_ik = np.interp( rx_tar, [rx_frame_min, rx_frame_max], [rik_ymin, rik_ymax] )

                rospy.loginfo('[Right Arm] Input Coor X: {0}, Y: {1}'.format(rx_tar, ry_tar))
                rospy.loginfo('[Right Arm] X_IK: {0:.2f}, Y_IK: {1:.2f}'.format(rx_ik, ry_ik))

                # kinematics.set_kinematics_pose("right_arm" , 2.0, **{ 'x': rx_ik, 'y': ry_ik, 'z': 0.65, 'roll': 90.00, 'pitch': 0.00, 'yaw': -20.00 })
                kinematics.set_kinematics_pose("right_arm" , 2.0, **{ 'x': rx_ik, 'y': ry_ik, 'z': 0.64, 'roll': 170.00, 'pitch': 20.00, 'yaw': 40.00 })
            else:
                rospy.logerr('[WS] X Frame target is out of range')

        else:
            rospy.logerr('[WS] Y Frame target is out of range')
        
    kinematics.kill_threads()

if __name__ == '__main__':
    main()
    