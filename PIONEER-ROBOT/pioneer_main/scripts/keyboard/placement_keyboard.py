#!/usr/bin/env python3

import yaml
import rospy
import rospkg
import numpy as np
from time import sleep
from std_msgs.msg import Bool
from geometry_msgs.msg import Point32
from geometry_msgs.msg import Pose2D
from pioneer_simulation.msg import Pose2DArray
from pioneer_kinematics.kinematics import Kinematics

class Placement_Keyboard:
    def __init__(self):
        np.set_printoptions(suppress=True)
        rospy.init_node('pioneer_placement_keyboard', anonymous=False)
        rospy.loginfo("[PK] Pioneer Align Keyboard - Running")

        rospack      = rospkg.RosPack()
        self.ws_path = rospack.get_path("pioneer_main") + "/config/thormang3_align_keyboard_ws.yaml"

        # Subscriber
        rospy.Subscriber("/pioneer/target/left_arm_point",      Point32,     self.left_arm_pos_callback)
        rospy.Subscriber("/pioneer/target/right_arm_point",     Point32,     self.right_arm_pos_callback)
        rospy.Subscriber("/pioneer/target/start",               Bool,        self.arm_start_callback)
        rospy.Subscriber("/pioneer/target/sync_arm",            Bool,        self.arm_sync_callback)
        rospy.Subscriber("/pioneer/target/grasp_keyboard",      Bool,        self.grip_key_callback)
        rospy.Subscriber("/pioneer/init_pose",                  Bool,        self.ini_pose_callback)
        rospy.Subscriber("/pioneer/placement/left_arm_points",  Pose2DArray, self.left_arm_points_callback)
        rospy.Subscriber("/pioneer/placement/right_arm_points", Pose2DArray, self.right_arm_points_callback)

        # Publisher

        # Variables
        self.kinematics = Kinematics()
        self.main_rate  = rospy.Rate(20)
        self.state      = None

        self.prev_lik,    self.prev_rik     = (), ()
        self.left_tar_x,  self.left_tar_y   = None, None
        self.right_tar_x, self.right_tar_y  = None, None
        self.prev_l_target                  = (None, None)
        self.prev_r_target                  = (None, None)
        self.left_points, self.right_points = None, None

    def left_arm_pos_callback(self, msg):
        self.left_tar_x = msg.x
        self.left_tar_y = msg.y

        self.lx_ik, self.ly_ik = self.left_arm_ik(self.left_tar_x, self.left_tar_y)

    def right_arm_pos_callback(self, msg):
        self.right_tar_x = msg.x
        self.right_tar_y = msg.y

        self.rx_ik, self.ry_ik = self.right_arm_ik(self.right_tar_x, self.right_tar_y)

    def arm_start_callback(self, msg):
        if msg.data == True:
            self.state = 'approach_keyboard'

    def arm_sync_callback(self, msg):
        if msg.data == True:
            self.state = 'sync_move_arms'

    def ini_pose_callback(self, msg):
        if msg.data == True:
            self.state = 'init_pose'

    def grip_key_callback(self, msg):
        if msg.data == True:
            self.state = 'grip_keyboard'

    def wait_robot(self, obj, msg):
        while obj.status_msg != msg:
            pass # do nothing

    def left_arm_points_callback(self, msg):
        group                = msg.name
        num_points           = len(msg.poses)
        left_points          = msg.poses
        self.ik_l_trajectory = [ self.left_arm_ik(pose.x, pose.y) for pose in left_points]

        # rospy.loginfo('[PA] Group: {}, Total_Points: {}, Points: {}'.format(group, num_points, left_points))
        # rospy.loginfo('[PA] Group: {}, Total_Points: {}, IK_Point: {}'.format(group, num_points, self.ik_l_trajectory))

    def right_arm_points_callback(self, msg):
        group                = msg.name
        num_points           = len(msg.poses)
        right_points         = msg.poses
        self.ik_r_trajectory = [ self.right_arm_ik(pose.x, pose.y) for pose in right_points]

        # rospy.loginfo('[PA] Group: {}, Total_Points: {}, Points: {}'.format(group, num_points, right_points))
        # rospy.loginfo('[PA] Group: {}, Total_Points: {}, IK_Points: {}'.format(group, num_points, self.ik_r_trajectory))

    def load_ws_config(self, arm):
        try:
            with open(self.ws_path, 'r') as f:
                aruco_ws = yaml.safe_load(f)
                rospy.loginfo('[PK] Loaded {} workspace'.format(arm))
            return aruco_ws[arm]

        except yaml.YAMLError as exc:
            print(exc)
            return None

    def parse_Yframe(self, data):
        y_frame_min = np.mean([ data['P2']['cy'], data['P3']['cy'] ], dtype=int)
        y_frame_max = np.mean([ data['P1']['cy'], data['P4']['cy'] ], dtype=int)

        ik_xmin = data['P1']['ik_x']
        ik_xmax = data['P2']['ik_x']
        return ( y_frame_min, y_frame_max, ik_xmin, ik_xmax )

    def parse_Xframe(self, data):
        x_frame_min = ( data['P1']['cx'], data['P2']['cx'] )
        x_frame_max = ( data['P3']['cx'], data['P4']['cx'] )

        ik_ymin_lower = data['P4']['ik_y'] #0.1
        ik_ymax_lower = data['P1']['ik_y'] #0.24
        ik_ymin_upper = data['P3']['ik_y'] #0.05
        ik_ymax_upper = data['P2']['ik_y'] #0.34
        return ( x_frame_min, x_frame_max, ik_ymax_lower, ik_ymax_upper, ik_ymin_lower, ik_ymin_upper)

    def move_arm(self, arm, x, y):
        zl, rl, pl, yl = 0.63, 150, -1, -29  # 0.63
        zr, rr, pr, yr = 0.635, -150, -1, 29 # 0.64

        if arm == "left_arm":
            self.kinematics.set_kinematics_pose(arm , 2.0, **{ 'x': x, 'y': y, 'z': zl, 'roll': rl, 'pitch': pl, 'yaw': yl })
            self.prev_lik = (x, y)
        elif arm == "right_arm":
            self.kinematics.set_kinematics_pose(arm , 2.0, **{ 'x': x, 'y': y, 'z': zr, 'roll': rr, 'pitch': pr, 'yaw': yr })
            self.prev_rik = (x, y)

    def ik_reference(self, left_ws, right_ws):
        if left_ws != None:
            yleft_data    = self.parse_Yframe(left_ws)
            self.ly_frame_min  = yleft_data[0]
            self.ly_frame_max  = yleft_data[1]
            self.ly_ws    = range(self.ly_frame_min, self.ly_frame_max+1)
            self.lik_xmin = yleft_data[2]
            self.lik_xmax = yleft_data[3]

            xleft_data          = self.parse_Xframe(left_ws)
            self.lx_min_b       = xleft_data[0][0]
            self.lx_min_a       = xleft_data[0][1]
            self.lx_max_a       = xleft_data[1][0]
            self.lx_max_b       = xleft_data[1][1]

            self.lik_ymax_lower = xleft_data[2] #0.24
            self.lik_ymax_upper = xleft_data[3] #0.34
            self.lik_ymin_lower = xleft_data[4] #0.1
            self.lik_ymin_upper = xleft_data[5] #0.05

        if right_ws != None:
            yright_data   = self.parse_Yframe(right_ws)
            self.ry_frame_min  = yright_data[0]
            self.ry_frame_max  = yright_data[1]
            self.ry_ws         = range(self.ry_frame_min, self.ry_frame_max+1)
            self.rik_xmin      = yright_data[2]
            self.rik_xmax      = yright_data[3]

            xright_data        = self.parse_Xframe(right_ws)
            self.rx_max_b      = xright_data[0][0]
            self.rx_max_a      = xright_data[0][1]
            self.rx_min_a      = xright_data[1][0]
            self.rx_min_b      = xright_data[1][1]

            self.rik_ymax_lower = xright_data[2] #0.24
            self.rik_ymax_upper = xright_data[3] #0.34
            self.rik_ymin_lower = xright_data[4] #0.1
            self.rik_ymin_upper = xright_data[5] #0.05

    def left_arm_ik(self, cx, cy):
        if cy in self.ly_ws:
            # Left IK X Target
            lx_ik = np.interp( cy, [self.ly_frame_min, self.ly_frame_max], [self.lik_xmax, self.lik_xmin] )

            # Mapping CX Frame
            lx_frame_min = np.interp( cy, [self.ly_frame_min, self.ly_frame_max], [self.lx_min_a, self.lx_min_b] )
            lx_frame_min = int( np.round(lx_frame_min, 0) )
            lx_frame_max = np.interp( cy, [self.ly_frame_min, self.ly_frame_max], [self.lx_max_a, self.lx_max_b] )
            lx_frame_max = int( np.round(lx_frame_max, 0) )

            # Mapping IK_Y
            lik_ymax     = np.interp( cy, [self.ly_frame_min, self.ly_frame_max], [self.lik_ymax_upper, self.lik_ymax_lower] )
            lik_ymax     = np.round(lik_ymax, 4)
            lik_ymin     = np.interp( cy, [self.ly_frame_min, self.ly_frame_max], [self.lik_ymin_upper, self.lik_ymin_lower] )
            lik_ymin     = np.round(lik_ymin, 4)

            lx_ws = range(lx_frame_min, lx_frame_max+1)
            if cx in lx_ws:
                # Left IK Y Target
                ly_ik = np.interp( cx, [lx_frame_min, lx_frame_max], [lik_ymax, lik_ymin] )

                return lx_ik, ly_ik
                # print()
                # rospy.loginfo('[Left Arm] Input Coor X: {0}, Y: {1}'.format(cx, cy))
                # rospy.loginfo('[Left Arm] X_IK: {0:.2f}, Y_IK: {1:.2f}'.format(self.lx_ik, self.ly_ik))
            else:
                return None, None
                # rospy.logerr('[Left Arm] X Frame target is out of range')
        else:
            return None, None
            # rospy.logerr('[Left Arm] Y Frame target is out of range')

    def right_arm_ik(self, cx, cy):
        if cy in self.ry_ws:
            # Right IK X Target
            rx_ik = np.interp( cy, [self.ry_frame_min, self.ry_frame_max], [self.rik_xmax, self.rik_xmin] )

            # Mapping CX Frame
            rx_frame_min = np.interp( cy, [self.ry_frame_min, self.ry_frame_max], [self.rx_min_a, self.rx_min_b] )
            rx_frame_min = int( np.round(rx_frame_min, 0) )
            rx_frame_max = np.interp( cy, [self.ry_frame_min, self.ry_frame_max], [self.rx_max_a, self.rx_max_b] )
            rx_frame_max = int( np.round(rx_frame_max, 0) )

            # Mapping IK_Y
            rik_ymax     = np.interp( cy, [self.ry_frame_min, self.ry_frame_max], [self.rik_ymax_upper, self.rik_ymax_lower] )
            rik_ymax     = np.round(rik_ymax, 4)
            rik_ymin     = np.interp( cy, [self.ry_frame_min, self.ry_frame_max], [self.rik_ymin_upper, self.rik_ymin_lower] )
            rik_ymin     = np.round(rik_ymin, 4)

            rx_ws = range(rx_frame_min, rx_frame_max+1)
            if cx in rx_ws:
                # Left IK Y Target
                ry_ik = np.interp( cx, [rx_frame_min, rx_frame_max], [rik_ymin, rik_ymax] )
                
                return rx_ik, ry_ik
                # print()
                # rospy.loginfo('[Right Arm] Input Coor X: {0}, Y: {1}'.format(cx, cy))
                # rospy.loginfo('[Right Arm] X_IK: {0:.2f}, Y_IK: {1:.2f}'.format(self.rx_ik, self.ry_ik))
            else:
                return None, None
                # rospy.logerr('[Right Arm] X Frame target is out of range')
        else:
            return None, None
            # rospy.logerr('[Right Arm] Y Frame target is out of range')

    def run(self):
        kinematics = self.kinematics

        # kinematics.publisher_(kinematics.module_control_pub,    "manipulation_module", latch=True)  # <-- Enable Manipulation mode
        # kinematics.publisher_(kinematics.send_ini_pose_msg_pub, "align_keyboard_pose", latch=True)
        # kinematics.publisher_(kinematics.en_align_key_pub,       True,                 latch=False) # <-- Enable Align Keboard mode
        # self.wait_robot(kinematics, "End Init Trajectory")

        # # set init head, torso, gripper
        # kinematics.set_joint_pos(['head_p', 'head_y', 'torso_y'], [30, 0, 0])
        # kinematics.set_gripper("left_arm", 0, 0)
        # kinematics.set_gripper("right_arm", 0, 0)
        # sleep(1)
        # kinematics.set_joint_pos(['l_arm_finger45_p', 'r_arm_finger45_p'], [180, 180])
        # sleep(1)
        # rospy.loginfo('[PK] Finish Init Head & Hand')

        # load config file
        left_ws  = self.load_ws_config('left_arm')
        right_ws = self.load_ws_config('right_arm')
        self.ik_reference(left_ws, right_ws)

        while not rospy.is_shutdown():
            if self.state == 'init_pose':
                rospy.loginfo('[PK] Robot State : {}'.format(self.state))

                if self.prev_lik and self.prev_rik:
                    self.lx_ik = self.prev_lik[0]
                    self.ly_ik = self.prev_lik[1] + 0.062
                    self.move_arm("left_arm" , self.lx_ik, self.ly_ik)

                    self.rx_ik = self.prev_rik[0]
                    self.ry_ik = self.prev_rik[1] - 0.062
                    self.move_arm("right_arm" , self.rx_ik, self.ry_ik)
                    sleep(2.5)
                    kinematics.publisher_(kinematics.send_ini_pose_msg_pub, "align_keyboard_pose", latch=False)
                    kinematics.publisher_(kinematics.en_align_key_pub, True, latch=False) # <-- Enable Align Keboard mode
                else:
                    kinematics.publisher_(kinematics.send_ini_pose_msg_pub, "align_keyboard_pose", latch=False)
                    kinematics.publisher_(kinematics.en_align_key_pub, True, latch=False) # <-- Enable Align Keboard mode

                self.lx_ik = self.ly_ik = self.rx_ik = self.ry_ik = None
                self.prev_lik = self.prev_rik = ()

                self.state = None

            elif self.state == 'approach_keyboard':
                rospy.loginfo('[PK] Robot State : {}'.format(self.state))

                sleep(1)

                if self.lx_ik != None and self.ly_ik != None \
                    and self.rx_ik != None and self.ry_ik != None:
                    print('move arm now')

                    self.ly_ik += 0.05
                    self.lx_ik -= 0.0 #0.025
                    self.move_arm("left_arm" , self.lx_ik, self.ly_ik)
                    self.ry_ik -= 0.05
                    self.rx_ik -= 0.0
                    self.move_arm("right_arm" , self.rx_ik, self.ry_ik)
                else:
                    rospy.logwarn('[PK] Robot arm singularities \n Please move keyboard to workspace')
                self.state = None

            elif self.state == 'grip_keyboard':
                rospy.loginfo('[PK] Robot State : {}'.format(self.state))

                if self.prev_lik and self.prev_rik:
                    self.lx_ik = self.prev_lik[0]
                    self.ly_ik = self.prev_lik[1] - 0.062
                    self.move_arm("left_arm" , self.lx_ik, self.ly_ik)

                    self.rx_ik = self.prev_rik[0]
                    self.ry_ik = self.prev_rik[1] + 0.062
                    self.move_arm("right_arm" , self.rx_ik, self.ry_ik)
                self.state = None

            elif self.state == 'sync_move_arms':
                rospy.loginfo('[PK] Robot State : {}'.format(self.state))
               
                # self.move_arm("right_arm" , self.rx_ik, self.ry_ik)
                # self.move_arm("left_arm"  , self.lx_ik, self.ly_ik)
                self.state = None
                
            # else:

            self.main_rate.sleep()
        kinematics.kill_threads()

if __name__ == '__main__':
    pk = Placement_Keyboard()
    pk.run()