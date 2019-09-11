#!/usr/bin/env python3

import yaml
import math
import rospy
import rospkg
import numpy as np
from time import sleep
from std_msgs.msg import Bool
from geometry_msgs.msg import Point32
from pioneer_kinematics.kinematics import Kinematics

rospack       = rospkg.RosPack()
ws_path       = rospack.get_path("pioneer_main") + "/config/thormang3_typing_ws.yaml"
keyboard_path = rospack.get_path("pioneer_main") + "/config/thormang3_keyboard_cfg.yaml"

np.set_printoptions(suppress=True)

def wait_robot(obj, msg):
    sleep(1)
    while obj.status_msg != msg:
        pass # do nothing

def keyboard_pos_callback(msg):
    global keyboard_pos
    # msg.z = theta rotation of keyboard
    keyboard_pos = (msg.x, msg.y, msg.z)

def load_ws_config(arm):
    try:
        with open(ws_path, 'r') as f:
            aruco_ws = yaml.safe_load(f)
            rospy.loginfo('[TY] Loaded {} workspace'.format(arm))
        return aruco_ws[arm]

    except yaml.YAMLError as exc:
        print(exc)
        return None

def load_keyboard_config():
    try:
        with open(keyboard_path, 'r') as f:
            return yaml.load(f)
            rospy.loginfo('[TY] Loaded Keyboard Config')

    except yaml.YAMLError as exc:
        print(exc)
        return None

def translate_2D_point(point, diff_point):
    return tuple(np.array(point) + diff_point)

def rotate_2D_point(origin, point, angle):
    """
    Rotate a point counterclockwise by a given angle around a given origin.
    The angle should be given in radians.
    """
    ox, oy = origin
    px, py = point

    qx = ox + math.cos(angle) * (px - ox) - math.sin(angle) * (py - oy)
    qy = oy + math.sin(angle) * (px - ox) + math.cos(angle) * (py - oy)
    return (qx, qy)

def calc_keyboard_frame_position():
    global keyboard_pos
    global l_1st_row, r_1st_row, l_2nd_row, r_2nd_row
    global l_3rd_row, r_3rd_row, l_4th_row, r_4th_row

    keyboard_cfg = load_keyboard_config()

    # load keyboard initial pose in frame coordinate
    init_keyb = (keyboard_cfg['aruco_ref']['cx'], keyboard_cfg['aruco_ref']['cy'])

    l_1st_row = {k: keyboard_cfg['_1st_row'][k] for k in ('1', '2', '3', '4', '5')}  
    r_1st_row = {k: keyboard_cfg['_1st_row'][k] for k in ('6', '7', '8', '9', '0')}  
    l_2nd_row = {k: keyboard_cfg['_2nd_row'][k] for k in ('q', 'w', 'e', 'r', 't')}  
    r_2nd_row = {k: keyboard_cfg['_2nd_row'][k] for k in ('y', 'u', 'i', 'o', 'p')}  
    l_3rd_row = {k: keyboard_cfg['_3rd_row'][k] for k in ('a', 's', 'd', 'f', 'g')}  
    r_3rd_row = {k: keyboard_cfg['_3rd_row'][k] for k in ('h', 'j', 'k', 'l', ';')}  
    l_4th_row = {k: keyboard_cfg['_4th_row'][k] for k in ('z', 'x', 'c', 'v', 'b')}  
    r_4th_row = {k: keyboard_cfg['_4th_row'][k] for k in ('n', 'm', ',', '.', '/')}

    # calculate diff position
    diff_keyboard = np.array(keyboard_pos[:-1]) - np.array(init_keyb)
    rospy.loginfo('[TY] Differentiate Keyboard: {}'.format(diff_keyboard))

    # update current position in frame coordinate
    # translate point
    l_1st_row = {key: translate_2D_point(val, diff_keyboard) for key, val in l_1st_row.items()}
    r_1st_row = {key: translate_2D_point(val, diff_keyboard) for key, val in r_1st_row.items()}
    l_2nd_row = {key: translate_2D_point(val, diff_keyboard) for key, val in l_2nd_row.items()}
    r_2nd_row = {key: translate_2D_point(val, diff_keyboard) for key, val in r_2nd_row.items()}
    l_3rd_row = {key: translate_2D_point(val, diff_keyboard) for key, val in l_3rd_row.items()}
    r_3rd_row = {key: translate_2D_point(val, diff_keyboard) for key, val in r_3rd_row.items()}
    l_4th_row = {key: translate_2D_point(val, diff_keyboard) for key, val in l_4th_row.items()}
    r_4th_row = {key: translate_2D_point(val, diff_keyboard) for key, val in r_4th_row.items()}

    theta = keyboard_pos[2]
    rospy.loginfo('[TY] Keyboard theta: {} degree'.format(theta))
    # rotation point
    if theta != 0:
        l_1st_row = {key: rotate_2D_point( keyboard_pos[:-1], val, math.radians(theta)) for key, val in l_1st_row.items()}
        r_1st_row = {key: rotate_2D_point( keyboard_pos[:-1], val, math.radians(theta)) for key, val in r_1st_row.items()}
        l_2nd_row = {key: rotate_2D_point( keyboard_pos[:-1], val, math.radians(theta)) for key, val in l_2nd_row.items()}
        r_2nd_row = {key: rotate_2D_point( keyboard_pos[:-1], val, math.radians(theta)) for key, val in r_2nd_row.items()}
        l_3rd_row = {key: rotate_2D_point( keyboard_pos[:-1], val, math.radians(theta)) for key, val in l_3rd_row.items()}
        r_3rd_row = {key: rotate_2D_point( keyboard_pos[:-1], val, math.radians(theta)) for key, val in r_3rd_row.items()}
        l_4th_row = {key: rotate_2D_point( keyboard_pos[:-1], val, math.radians(theta)) for key, val in l_4th_row.items()}
        r_4th_row = {key: rotate_2D_point( keyboard_pos[:-1], val, math.radians(theta)) for key, val in r_4th_row.items()}


def calc_keyboard_ik_position():
    global l_1st_row, r_1st_row, l_2nd_row, r_2nd_row
    global l_3rd_row, r_3rd_row, l_4th_row, r_4th_row

    global ikl_1st_row, ikr_1st_row, ikl_2nd_row, ikr_2nd_row
    global ikl_3rd_row, ikr_3rd_row, ikl_4th_row, ikr_4th_row

    ikl_1st_row = {key: left_arm_ik(val[0], val[1]) for key, val in l_1st_row.items()}
    ikl_2nd_row = {key: left_arm_ik(val[0], val[1]) for key, val in l_2nd_row.items()}
    ikl_3rd_row = {key: left_arm_ik(val[0], val[1]) for key, val in l_3rd_row.items()}
    ikl_4th_row = {key: left_arm_ik(val[0], val[1]) for key, val in l_4th_row.items()}

    # IK Conversion
    ikr_1st_row = {key: right_arm_ik(val[0], val[1]) for key, val in r_1st_row.items()}
    ikr_2nd_row = {key: right_arm_ik(val[0], val[1]) for key, val in r_2nd_row.items()}
    ikr_3rd_row = {key: right_arm_ik(val[0], val[1]) for key, val in r_3rd_row.items()}
    ikr_4th_row = {key: right_arm_ik(val[0], val[1]) for key, val in r_4th_row.items()}

def parse_Yframe(data):
    y_frame_min = np.mean([ data['P2']['cy'], data['P3']['cy'] ], dtype=int)
    y_frame_max = np.mean([ data['P1']['cy'], data['P4']['cy'] ], dtype=int)

    # y_frame_min = ( data['P2']['cy'], data['P3']['cy'] )
    # y_frame_max = ( data['P1']['cy'], data['P4']['cy'] )

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

def ik_ref(left_ws, right_ws):
    global ly_ws, ly_frame_min, ly_frame_max, lik_xmin, lik_xmax
    global lx_min_b, lx_min_a, lx_max_a, lx_max_b
    global lik_ymax_lower, lik_ymax_upper, lik_ymin_lower, lik_ymin_upper

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

        lik_ymax_lower = xleft_data[2]
        lik_ymax_upper = xleft_data[3]
        lik_ymin_lower = xleft_data[4]
        lik_ymin_upper = xleft_data[5]

    global ry_ws, ry_frame_min, ry_frame_max, rik_xmin, rik_xmax
    global rx_min_b, rx_min_a, rx_max_a, rx_max_b
    global rik_ymax_lower, rik_ymax_upper, rik_ymin_lower, rik_ymin_upper

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

        rik_ymax_lower = xright_data[2]
        rik_ymax_upper = xright_data[3]
        rik_ymin_lower = xright_data[4]
        rik_ymin_upper = xright_data[5]

def left_arm_ik(cx, cy):
    global ly_ws, ly_frame_min, ly_frame_max, lik_xmin, lik_xmax
    global lx_min_b, lx_min_a, lx_max_a, lx_max_b
    global lik_ymax_lower, lik_ymax_upper, lik_ymin_lower, lik_ymin_upper

    if cy in ly_ws:
        # Mapping CY Frame
        # ly_frame_min = np.interp( cy, [ly_frame_min, ly_frame_max], [ly_min_a, ly_min_b] )
        # ly_frame_min = int( np.round(ly_frame_min, 0) )

        # Left IK X Target
        lx_ik = np.interp( cy, [ly_frame_min, ly_frame_max], [lik_xmax, lik_xmin] )

        # Mapping CX Frame
        lx_frame_min = np.interp( cy, [ly_frame_min, ly_frame_max], [lx_min_a, lx_min_b] )
        lx_frame_min = int( np.round(lx_frame_min, 0) )
        lx_frame_max = np.interp( cy, [ly_frame_min, ly_frame_max], [lx_max_a, lx_max_b] )
        lx_frame_max = int( np.round(lx_frame_max, 0) )

        # Mapping IK_Y
        lik_ymax     = np.interp( cy, [ly_frame_min, ly_frame_max], [lik_ymax_upper, lik_ymax_lower] )
        lik_ymax     = np.round(lik_ymax, 4)
        lik_ymin     = np.interp( cy, [ly_frame_min, ly_frame_max], [lik_ymin_upper, lik_ymin_lower] )
        lik_ymin     = np.round(lik_ymin, 4)

        lx_ws = range(lx_frame_min, lx_frame_max+1)
        if cx in lx_ws:
            # Left IK Y Target
            ly_ik = np.interp( cx, [lx_frame_min, lx_frame_max], [lik_ymax, lik_ymin] )

            return (lx_ik, ly_ik)
        else:
            return (None, None)
    else:
        return (None, None)

def right_arm_ik(cx, cy):
    global ry_ws, ry_frame_min, ry_frame_max, rik_xmin, rik_xmax
    global rx_min_b, rx_min_a, rx_max_a, rx_max_b
    global rik_ymax_lower, rik_ymax_upper, rik_ymin_lower, rik_ymin_upper

    if cy in ry_ws:
        # Left IK X Target
        rx_ik = np.interp( cy, [ry_frame_min, ry_frame_max], [rik_xmax, rik_xmin] )

        # Mapping CX Frame
        rx_frame_min = np.interp( cy, [ry_frame_min, ry_frame_max], [rx_min_a, rx_min_b] )
        rx_frame_min = int( np.round(rx_frame_min, 0) )
        rx_frame_max = np.interp( cy, [ry_frame_min, ry_frame_max], [rx_max_a, rx_max_b] )
        rx_frame_max = int( np.round(rx_frame_max, 0) )

        # Mapping IK_Y
        rik_ymax     = np.interp( cy, [ry_frame_min, ry_frame_max], [rik_ymax_upper, rik_ymax_lower] )
        rik_ymax     = np.round(rik_ymax, 4)
        rik_ymin     = np.interp( cy, [ry_frame_min, ry_frame_max], [rik_ymin_upper, rik_ymin_lower] )
        rik_ymin     = np.round(rik_ymin, 4)

        rx_ws = range(rx_frame_min, rx_frame_max+1)
        if cx in rx_ws:
            # Left IK Y Target
            ry_ik = np.interp( cx, [rx_frame_min, rx_frame_max], [rik_ymin, rik_ymax] )
            return (rx_ik, ry_ik)
        else:
            return (None, None)
    else:
        return (None, None)

def check_singularities(x, y):
    if x is not None and y is not None:
        return True
    else:
        return False

def process(kinematics, k):
    arm  = None

    if k in ikl_1st_row.keys():
        x, y = ikl_1st_row[k]
        if check_singularities(x, y):
            arm  = "left_arm"
            x -= 0.008
            y += 0.006
    elif k in ikl_2nd_row.keys():
        x, y = ikl_2nd_row[k]
        if check_singularities(x, y):
            arm  = "left_arm"
            x -= 0.004
            y += 0.002
    elif k in ikl_3rd_row.keys():
        x, y = ikl_3rd_row[k]
        if check_singularities(x, y):
            arm  = "left_arm"
            y += 0.004
    elif k in ikl_4th_row.keys():
        x, y = ikl_4th_row[k]
        if check_singularities(x, y):
            arm  = "left_arm"
            x += 0.002
            y += 0.002

    elif k in ikr_1st_row.keys():
        x, y = ikr_1st_row[k]
        if check_singularities(x, y):
            arm  = "right_arm"
            x -= 0.015
    elif k in ikr_2nd_row.keys():
        x, y = ikr_2nd_row[k]
        if check_singularities(x, y):
            arm  = "right_arm"
            x -= 0.013
    elif k in ikr_3rd_row.keys():
        x, y = ikr_3rd_row[k]
        if check_singularities(x, y):    
            arm  = "right_arm"
            x -= 0.01
    elif k in ikr_4th_row.keys():
        x, y = ikr_4th_row[k]
        if check_singularities(x, y):
            arm  = "right_arm"
            x -= 0.006

    if arm != None:
        rospy.loginfo('[TY] {}, X : {} Y : {} '.format(arm, x, y))
        if arm == "left_arm":
            z = 0.72
        elif arm == "right_arm":
            z = 0.73
        move_arm(kinematics, arm, x, y, z)
    else:
        rospy.logwarn('[TY] Key : {} is out of workspace'.format(k))

def move_arm(kinematics, arm, x, y, z):
    global prev_ik, last_arm
    rl, pl, yl = 10.0,  0.0, 0.0
    rr, pr, yr = -10.0, 0.0, 0.0

    if arm == "left_arm":         
        kinematics.set_kinematics_pose(arm , 2.0, **{ 'x': x, 'y': y, 'z': z, 'roll': rl, 'pitch': pl, 'yaw': yl })
    elif arm == "right_arm":
        kinematics.set_kinematics_pose(arm , 2.0, **{ 'x': x, 'y': y, 'z': z, 'roll': rr, 'pitch': pr, 'yaw': yr })
       
    prev_ik  = (x, y, z)
    last_arm = arm

def main():
    rospy.init_node('pioneer_main_typing', anonymous=False)
    rospy.loginfo("[TY] Pioneer Main Typing - Running")

    rospy.Subscriber("/pioneer/aruco/keyboard_position", Point32, keyboard_pos_callback)

    # Kinematics
    kinematics = Kinematics()
    # kinematics.publisher_(kinematics.module_control_pub,    "manipulation_module", latch=True)  # <-- Enable Manipulation mode
    # kinematics.publisher_(kinematics.send_ini_pose_msg_pub, "typing_pose",         latch=True)
    # kinematics.publisher_(kinematics.en_typing_pub,          True,                 latch=False) # <-- Enable Typing mode
    # wait_robot(kinematics, "End Init Trajectory")

    # # Typing Init Pose
    # kinematics.set_joint_pos(['head_p', 'head_y', 'torso_y'], [30, 0, 0])
    # kinematics.set_gripper("left_arm", 0, 0)
    # kinematics.set_gripper("right_arm", 0, 0)
    # sleep(2)
    
    # # Set Finger
    # kinematics.set_joint_pos(['l_arm_index_p', 'l_arm_thumb_p'], [-86, -70])
    # kinematics.set_joint_pos(['r_arm_index_p', 'r_arm_thumb_p'], [-86, -70])
    # rospy.loginfo('[TY] Finish Init Head & Hand')

    main_rate = rospy.Rate(20)

    global keyboard_pos
    keyboard_pos = None

    # waiting aruco position from recorder
    while not rospy.is_shutdown():
        while keyboard_pos == None:
            pass
        sleep(2)
        break

    # load & calculate keyboard frame position
    calc_keyboard_frame_position()

    # load IK config file
    left_ws  = load_ws_config('left_arm')
    right_ws = load_ws_config('right_arm')
    ik_ref(left_ws, right_ws)

    # convert keyboard frame to ik position
    calc_keyboard_ik_position()

    while not rospy.is_shutdown():
        key = input("Input key : ")

        if key == 'exit':
            rospy.loginfo('[TY] Pioneer Typing Exit')
            break
        elif key == 'init':
            kinematics.publisher_(kinematics.send_ini_pose_msg_pub, "typing_pose", latch=False)
        elif key == 'typing':
            move_arm(kinematics, last_arm, prev_ik[0], prev_ik[1], prev_ik[2]-0.02)
            sleep(3)
            move_arm(kinematics, last_arm, prev_ik[0], prev_ik[1], prev_ik[2]+0.02)
        else:
            process(kinematics, key)

        main_rate.sleep()

    kinematics.kill_threads()

if __name__ == '__main__':
    main()
    