#!/usr/bin/python3

import sys
import math
import rospy
import signal
import numpy as np
from time import sleep
import matplotlib.pyplot as plt

from sklearn.cluster import KMeans
from sklearn.externals import joblib
from sklearn import cluster

from std_msgs.msg import String, Bool
from thormang3_tts_msgs.msg import mp3
from robotis_controller_msgs.msg import StatusMsg
from sensor_msgs.msg import JointState , LaserScan
from pioneer_motor.motor import Motor
from pioneer_kinematics.kinematics import Kinematics


class magic():
    def __init__(self):
        
        print('start')
        self.kinematics = Kinematics()
        self.motor = Motor("Thormang3_Wolf")

        self.rate = rospy.Rate(10)

        self.speech = rospy.Publisher("/robotis/sensor/text_to_speech",String,queue_size=10)
        # self.speech = rospy.Publisher("/robotis/sensor/update_tts",String,queue_size=10)
        self.music = rospy.Publisher("/robotis/sensor/music",mp3,queue_size=10)
        self.music_stop = rospy.Publisher('/robotis/sensor/stop_music',     Bool,queue_size = 10)
        self.sensor_list = None
        self.pub_gripper = rospy.Publisher('/robotis/gripper/joint_pose_msg',JointState,queue_size=10)
        #base_ini
        self.base_ini = rospy.Publisher("/robotis/base/ini_pose",String,queue_size=10)
        
        for _ in range(4):
            self.base_ini.publish('ini_pose')
            self.rate.sleep()
        self.end_action()
        sleep(1)
        #print("start") 

        #ini_arm
        self.kinematics.publisher_(self.kinematics.module_control_pub, "manipulation_module", latch=True)
        self.kinematics.publisher_(self.kinematics.module_control_pub, "gripper_module", latch=True)
        self.kinematics.publisher_(self.kinematics.send_ini_pose_msg_pub, "ini_pose", latch=True)
        self.end_action()
        self.left_arm_ini_pos = self.kinematics.get_kinematics_pose('left_arm')
        
        self.kinematics.publisher_(self.kinematics.module_control_pub, "head_control_module", latch=True)
        #sleep(0.5)

        #move head
        self.head_postition = rospy.Publisher('/robotis/head_control/set_joint_states',JointState,queue_size=10)
        sleep(0.5)
        
        #print('moved')

        self.sensor_list = None
        self.deg = float(60)
        self.deg_30 = float(30 *math.pi /180)
        self.head_pan = np.radians(12)
        # self.head_pan = float(23 * math.pi / 180)
        self.head_tilt = float(36 * math.pi / 180)

        self.student_pan = np.radians(36)
        self.student_tilt = np.radians(-12)

        self.J_pan = np.radians(36)
        self.J_tilt = np.radians(-28)
        #sleep(1)
        #print('finished')
        # nan = None
        # self.calibration(ranges)
    
    def run(self):
        self.state = 'first_scene'
        # self.state = 'first_step'
        # self.state = ''
        rospy.Subscriber('/robotis/sensor/scan_filtered', LaserScan, self.sensor_callback)
        while (True):
            if self.sensor_list is not None :
                break
        
        temp_mp3 = mp3()
        temp_mp3.file_name = '100) open'
        temp_mp3.volume = 10
        self.music.publish(temp_mp3)

        input('press to start')

        # self.move_head()
        # sleep(2)



        while not rospy.is_shutdown():
            if self.state == 'first_scene':
                self.text_to_speech("1) Welcome everyone, I am Thormang.")
                sleep(0.5)
                self.text_to_speech("2) Today I am going to show you a magic trick that I have laerned recently")
                sleep(0.5)
                self.text_to_speech("3) As you can see, there are three paper bags in front of me.")
                sleep(0.5)
                self.text_to_speech("4) Later, I will put this wooden base with a sharp nail into one of the paper bags ")
                sleep(0.5)
                self.text_to_speech("74) and then swap around the position of the bags")
                sleep(0.5)
                self.text_to_speech("5) To make sure we wont find out the bag is empty by feeling the weight, so I prepared two wooden bases without nails.")
                sleep(0.5)
                input('press to look student')
                self.move_head(self.student_pan,self.student_tilt)
                sleep(0.5)
                self.text_to_speech("6) Student, please help me put these two wooden bases into two paper bags and close it.")
                sleep(0.5)
                input("after student move")
                self.move_head(self.head_pan,self.head_tilt)
                self.state = 'second_scene'
            elif self.state == 'second_scene':
                input("press to look student again")
                self.move_head(self.student_pan,self.student_tilt)
                sleep(0.5)
                self.text_to_speech("7) student, please ask an audience member to examine the nail and make sure it is real")
                sleep(1)
                self.move_head(0,0)
                input('wait for check the nail')
                self.text_to_speech("8) So, audience volunteer can you confirm to the rest of the audience that the nail is indeed real?")
                input('after responced ,press to continue')
                
                self.text_to_speech("9) Okay, thank you")
                sleep(0.5) 
                self.move_head(self.student_pan,self.student_tilt)
                sleep(0.5)
                self.text_to_speech("77) student please place the nail inside the last bag and close it.")
                sleep(1.5)
                self.move_head(0,0)
                self.text_to_speech("10) And everyone, please pay attention and remember which bag contains the nail.")
                sleep(0.5)
                
                sleep(1)
                input('wait for student action , press to continue')
                #self.move_head(0,0)
                self.state = "third_scene"

            elif self.state == "third_scene":
                #input('finished swap ,press to continue')
                sleep(1)
                self.text_to_speech("11) Now, I need another volunteer.")
                input('press to continue')
                self.text_to_speech("12) Thank you for volunteering ")
                sleep(0.5)
                self.text_to_speech("79) now , do you remember which bag contains the nail ?")
                input('after volunteer answered , press to continue')
                
                self.move_head(self.head_pan,self.head_tilt)
                sleep(2)
                # ranges = self.sensor_list [:]
                # self.calibration(ranges)
                while (True):
                    ranges = self.sensor_list [:]
                    self.calibration(ranges)
                    det = input('detect well ? (y/n)')
                    if det == "y":
                        break

                self.right_p1_up()
                self.move_head(0,0)
                self.text_to_speech("13) You mean this one , right?")
                
                input('press to continue')
                self.text_to_speech("14) To make sure that I wont be able to know which bag contains the nail")
                sleep(0.5)
                
                self.text_to_speech("15) the student will start swapping the bags around .")
                sleep(0.5)
                
                #input('press to continue')
                self.text_to_speech("60) When I say START the student will begin, and volunteer, you can shout out STOP anytime you want.")
                sleep(0.5)
                self.kinematics.publisher_(self.kinematics.send_ini_pose_msg_pub, "ini_pose", latch=True)
                self.end_action()

                self.move_head(self.head_pan,self.head_tilt)
                sleep(0.2)
                self.text_to_speech("16) Lets START!")
                self.music_stop.publish(True)
                sleep(0.5)
                temp_mp3 = mp3()
                temp_mp3.file_name = '200) start game'
                temp_mp3.volume = 15
                self.music.publish(temp_mp3)
                input('after swap ,press to detect the bag position')
                self.text_to_speech("17) Okay, now I need some time to feel where the nail is.")
                sleep(0.5)

                self.state = 'fourth_scene'

            elif self.state == 'fourth_scene':
                
                sleep(2)
                
                while (True):
                    ranges = self.sensor_list [:]
                    self.calibration(ranges)
                    det = input('detect well ? (y/n)')
                    if det == "y":
                        break
                
                self.p2_up()
                sleep(2)
                self.p1_up()
                sleep(2)
                self.p3_up()
                sleep(2)
                self.p2_up()
                
                point_back = self.kinematics.get_kinematics_pose('left_arm').copy()
                point_back["x"] = 0.4
                self.kinematics.set_kinematics_pose(group_name="left_arm",time=3,**point_back)
                self.end_action()
                
                self.kinematics.publisher_(self.kinematics.send_ini_pose_msg_pub, "ini_pose", latch=True)
                self.end_action()

                
                self.move_head(0,0)
                self.text_to_speech("18) Ohh, I am 100 percent sure where the nail is. So I would like to play something much more interesting.")
                sleep(2)
                self.move_head(self.J_pan,0)
                self.text_to_speech("19) Jacky")
                
                sleep(0.5)
                input('jacky come to the stage')
                self.move_head(self.J_pan,self.J_tilt)
                self.text_to_speech("20) as the supervisor of your lab, do you believe in my talent and your students ability?")
                input('after Jacky worlds ,press to continue')
                self.text_to_speech("61) oh , im happy to hear that !")

                self.text_to_speech("21) Since you trust us so much ")
                sleep(0.2)
                self.text_to_speech("66) we can raise the stakes.")
                sleep(0.7)
                self.text_to_speech("65) How about we use your hand to press down on the paper bags? ")
                input("jacky's time")
                
                self.p2_up()
                self.text_to_speech("80) Please give me your hand, Jacky")
                sleep(0.5)
                input('press to continue')
                self.move_head(0,0)
                self.text_to_speech("22) volunteer, choose a bag you think there's no nail inside? Left, Right, or Middle?")
                sleep(0.5)
                self.text_to_speech("62) Choose wisely, we wouldn’t want our dear professor to get hurt, right ?")
                self.state = "first_step"
            elif self.state == 'first_step':
                situation = input('put the situation (p3_l , p3_r , p1_l , p1_r) >> (1,2,3,4) :')
                self.move_head(self.head_pan,self.head_tilt)
                # while (True):
                    
                #     sleep(0.5)
                #     ranges = self.sensor_list [:]
                #     self.calibration(ranges)
                #     det = input('detect well ? (y/n)')
                #     if det == "y":
                #         break

                if situation == '1' : #"p3_l":
                    self.text_to_speech("23) Okay, left.")
                    
                    self.p3_up()
                    self.speech.publish("24) Safe!")
                    sleep(0.2)
                    self.DownAndUp()
                    
                    self.last = 'p1'
                elif situation == "2" : #"p3_r":
                    self.text_to_speech("25) Okay, right.")
                    
                    self.p3_up()
                    self.speech.publish("24) Safe!")
                    sleep(0.2)
                    self.DownAndUp()
                    
                    self.last = 'p1'
                elif situation == "3" : #"p1_l":
                    self.text_to_speech("23) Okay,left.")
                    
                    self.p1_up()
                    self.speech.publish("24) Safe!")
                    sleep(0.2)
                    self.DownAndUp()
                    
                    self.last = 'p3'
                elif situation == "4" : #"p1_r":
                    self.text_to_speech("25) Okay, right.")
                    
                    self.p1_up()
                    self.speech.publish("24) Safe!")
                    sleep(0.2)
                    self.DownAndUp()
                    
                    self.last = 'p3'
                else:
                    print("input should be 1 , 2 , 3 ,4 >> (p3_l , p3_r , p1_l , p1_r) ")
                    continue
                self.move_head(self.J_pan,self.J_tilt)
                self.text_to_speech("28) Jacky, dont be that nervous, dont you trust me?")
                sleep(0.5)
                input('press to continue')
                self.text_to_speech("29) I am the only hope of graduation for your students")
                sleep(0.7)
                self.text_to_speech("68) they won't disappoint you , trust me !")
                sleep(0.5)
                self.state = 'second_step'
                # if self.sensor_list is not None:
                #     #print(self.sensor_list)
                #     ranges = self.sensor_list [:]
                #     self.state = 'second_step'
                #     #print('first finished')
                #     # break

            elif self.state == 'second_step':
                #input("press to continue")
                #self.p2_up()
                input('press to continue')
                self.move_head(0,0)
                sleep(0.3)
                self.text_to_speech("30) Hey, volunteer, what's your next choice? Left or Right?")

                l_r = input('left or right  ( l , r ) : ')
                if l_r == "l":
                    
                    self.speech.publish("31) okay, left")
                else :
                    self.speech.publish("32) okay, right")
                
                if self.last == 'p1':
                    self.p1_up()
                elif self.last == 'p3':
                    self.p3_up()
                
                self.music_stop.publish(True)
                sleep(0.2)
                ################################################################################################################################
                ###########thormang3_crash#############
                temp_mp3 = mp3()
                temp_mp3.file_name = '101) restart'
                temp_mp3.volume = 25
                self.music.publish(temp_mp3)
                sleep(0.2)
                self.motor.publisher_(self.motor.module_control_pub, "none", latch=True)
                self.motor.set_joint_states(['all'], False)
                rospy.loginfo('[Magic] Turn off torque')
                ###################################
                sleep(1)
                self.speech.publish("41) re re re staaar rrr t teng ")
                sleep(2)
                input ("trigger")
                ##################################
                self.motor.set_joint_states(['all'], True)
                rospy.loginfo('[Magic] Turn on torque')

                ###############
                self.kinematics.publisher_(self.kinematics.module_control_pub, "manipulation_module", latch=True)
                self.kinematics.publisher_(self.kinematics.module_control_pub, "gripper_module", latch=True)
                self.kinematics.publisher_(self.kinematics.module_control_pub, "head_control_module", latch=True)
                temp_mp3 = mp3()
                temp_mp3.file_name = '201) start game'
                temp_mp3.volume = 10
                self.music.publish(temp_mp3)
                sleep(4)
                self.speech.publish("38) Finished rebooting ")
                # self.kinematics.publisher_(self.kinematics.send_ini_pose_msg_pub, "ini_pose", latch=True)
                # self.end_action() 
                ################################################################################################################################
                if self.last == 'p1':
                    self.p1_up()
                elif self.last == 'p3':
                    self.p3_up()
                self.speech.publish("39) Okay , this one ?")
                sleep(0.5)
                self.speech.publish("40) but i dont think this one is safe ")
                sleep(2)
                
                self.p2_up()
                sleep(0.2)
                self.move_head(self.J_pan,self.J_tilt)
                self.speech.publish("33) Jacky , do you trust me? ")
                sleep(2.5)
                input('press to continue')
                self.music_stop.publish(True)
                temp_mp3 = mp3()
                temp_mp3.file_name = '306) feel sad'
                temp_mp3.volume = 15
                self.music.publish(temp_mp3)
                sleep(7)
                self.text_to_speech("70) why? i only made one mistake ")
                sleep(0.2)
                self.text_to_speech("71) why dont you give me another chance")
                self.speech.publish("50) well ")
                sleep(0.5)
                self.text_to_speech("72) but i dont care !")
                sleep(0.2)
                self.DownAndUp()
                # self.music_stop.publish(True)
                # temp_mp3 = mp3()
                # temp_mp3.file_name = '300) start game'
                # temp_mp3.volume = 15
                # self.music.publish(temp_mp3)
                sleep(0.5)
                self.move_head(0,0)
                self.state = 'third_step'
                # # calculating position
                # self.calibration(ranges)
                # #print("second_finished")
                # self.state = 'third_step'
                # #break
            elif self.state == 'third_step' :
                sleep(0.5)
                self.text_to_speech("34) Yeah, got you! HA HA HA , Don't be that scared")
                sleep(0.5)
                self.text_to_speech("69) I know what I am doing")
                sleep(0.5)
                input('grab the bag')
                self.text_to_speech("35) So, you might be really curious where the actual nail is, right?")
                sleep(0.5)
                
                #input('press to continue')

                if self.last == 'p1':
                    self.p1_up()
                elif self.last == 'p3':
                    self.p3_up()

                self.grab_bag()
                sleep(0.5)
                
                
                input('press to make thormann grip off')
                self.grip_off('l_arm_grip')

                # input('press to continue')
                # self.text_to_speech("")

                input('press to continue')
                self.move_head(0,0)
                self.kinematics.publisher_(self.kinematics.send_ini_pose_msg_pub, "ini_pose", latch=True)
                self.end_action()
                self.grip_on('l_arm_grip')
                self.text_to_speech("37) Thanks for your attention, I am Thormang wolf from ERC lab, see you next time")
                
                        
                print("End Magic")
                self.kinematics.kill_threads()
                break
                

            
            rospy.Rate(60).sleep()
    
    def sensor_callback(self,msg):
        self.sensor_list = np.array(msg.ranges)

    def read_sensor(self):
        
        rospy.Subscriber('/robotis/sensor/scan_filtered', LaserScan, self.sensor_callback)

        while not rospy.is_shutdown():
            if self.sensor_list is not None:
                break
        #     # else:
        #     #     print(self.sensor_list)
            self.rate.sleep()

    def calibration(self,ranges,deg = 60,threshold = 0.78):
        nan = None
        #print(len(ranges))
        calibration = []
        #print(self.deg_30)
        per_deg = (self.deg/len(ranges) )* (math.pi/180)
        #print(per_deg)
        for i in range(len(ranges)):
            if ranges[i] == None :
                pass
            else:
                y = ranges[i] * math.cos(i*per_deg - (self.deg_30 - self.head_pan) ) * math.cos(self.head_tilt) 
                
                if y < threshold :
                    x = -ranges[i] * math.sin( (self.deg_30 - self.head_pan) - i*per_deg) * math.cos(self.head_tilt)
                    calibration.append([round(x,5),round(y,5),i])
        calibration = np.array(calibration)
        #print(len(calibration))
        #print(calibration)
        mean = sum(calibration)/len(calibration)

        valid = []
        for i in range(len(calibration)):
            if calibration[i][1] < mean[1]:
                valid.append(calibration[i])
        valid = np.array(valid)
        #print(valid)
        
        ##K-means n_clusters=3
        estimator = KMeans(n_clusters=3)
        res = estimator.fit_predict(valid[:,0:2])
        #print(res)
        lable_pred = estimator.labels_
        #print (lable_pred)
        entroids = estimator.cluster_centers_
        data = entroids[:]
        data = np.sort(data,axis = 0)
        print (data)

        

        self.point1 = {'x':0.52,'y':data[0,0]+0.02,'z':1.05,'roll':0,'pitch':0,'yaw':-25}
        self.point2 = {'x':0.52,'y':data[1,0]+0.03,'z':1.05,'roll':0,'pitch':0,'yaw':0}
        self.point3 = {'x':0.52,'y':data[2,0]+0.07,'z':1.05,'roll':0,'pitch':0,'yaw':30}
        self.right_point ={'x':0.52,'y': (data[0,0]-0.06),'z':1.00,'roll':0,'pitch':20,'yaw':30}
        if self.right_point["y"] > 0.05:
            self.right_point["y"] = 0 
        print("point1 : y = ",self.point1["y"] )
        print("point2 : y = ",self.point2["y"] )
        print("point3 : y = ",self.point3["y"] )
        print("right_y : ",self.right_point["y"])

        # plt.figure(figsize=(12,6))
        # plt.plot(calibration[:,0],calibration[:,1],'bo')
        # plt.plot(valid[:,0],valid[:,1],'go')
        # plt.plot(data[:,0],data[:,1],'ro')
        # plt.show()
    
    def text_to_speech(self,worlds):
        self.speech.publish(worlds)
        sleep(3.5)

    def move_head(self,pan,tilt):
    
        msg = JointState()
        msg.name = ['head_y','head_p']
        msg.position = [pan ,tilt]
        msg.velocity = [0]
        msg.effort = [0]
        
        msg.header.frame_id = ''
        msg.header.stamp.secs = 0
        msg.header.stamp.nsecs= 0
        for i in range(4):
            msg.header.seq = i
            self.head_postition.publish(msg)
            self.rate.sleep()
        

    def end_action(self):
        sleep(0.5)
        while not self.kinematics.status_msg == 'End Trajectory' :
            sleep(1)
            if self.kinematics.status_msg == 'Finish Init Pose':
                break
        sleep(0.5)
    def right_p1_up(self):
        self.kinematics.set_kinematics_pose(group_name="right_arm",time=3,**self.right_point) #x=0.52,y=0,z=1.1,roll=0,pitch=0,yaw=-40)
        self.end_action()

    def p1_up(self):
        self.kinematics.set_kinematics_pose(group_name="left_arm",time=3,**self.point1) #x=0.52,y=0,z=1.1,roll=0,pitch=0,yaw=-40)
        self.end_action()

    def p2_up(self):
        self.kinematics.set_kinematics_pose(group_name="left_arm",time=3,**self.point2) #x=0.52,y=0.2,z=1.1,roll=0,pitch=0,yaw=0)
        self.end_action()

    def p3_up(self):
        self.kinematics.set_kinematics_pose(group_name="left_arm",time=3,**self.point3) #x=0.52,y=0.4,z=1.1,roll=0,pitch=0,yaw=40)
        self.end_action()

    def grab_bag(self):
        #sleep(2)
        self.grip_off('l_arm_grip')
        self.end_action()
        point = self.kinematics.get_kinematics_pose('left_arm').copy()
        point_down = point.copy()
        point_down["z"] = 0.85
        point["x"] = 0.40
        point["z"] = 1.05
        point_ini = self.left_arm_ini_pos.copy()
        point_ini["z"] = 0.92
        left_side = {'x':0.28,'y': 0.55,'z':0.950,'roll':0,'pitch':0,'yaw':30.0}
        # right_center = {'x':0.36,'y': -0.03,'z':0.820,'roll':0,'pitch':0,'yaw':60.0}
        # right_split = {'x':0.32,'y': -0.16,'z':0.820,'roll':0,'pitch':0,'yaw':60.0}
        # right_up = {'x':0.4,'y': -0.02,'z':0.82,'roll':0,'pitch':0,'yaw':20}
        #print('point_down',point_down)
        #print('point',point)
        self.kinematics.set_kinematics_pose(group_name="left_arm",time=2,**point)
        self.end_action()

        point["z"] = 0.85
        self.kinematics.set_kinematics_pose(group_name="left_arm",time=2,**point)
        self.end_action()

        self.kinematics.set_kinematics_pose(group_name="left_arm",time=2,**point_down)
        self.end_action()
        self.grip_on('l_arm_grip')
        # sleep(0.2)
        self.kinematics.set_kinematics_pose(group_name="left_arm",time=2,**point)
        self.end_action()

        # self.kinematics.set_kinematics_pose(group_name="left_arm",time=3,**point_ini)
        # self.end_action()
        self.move_head(self.J_pan,self.J_tilt)
        self.text_to_speech("36) Jacky , please help me to check what is in the bag.")
        ##grab bag to the center
        self.kinematics.set_kinematics_pose(group_name="left_arm",time=4,**left_side)
        self.end_action()

        ##split

        # self.grip_off("r_arm_grip")
        # self.kinematics.set_kinematics_pose(group_name="right_arm",time=3,**right_center)
        # self.end_action()
        # self.grip_on("r_arm_grip")
        # self.kinematics.set_kinematics_pose(group_name="right_arm",time=3,**right_split)
        # self.end_action()
        # self.grip_half()

        # self.kinematics.set_kinematics_pose(group_name="right_arm",time=3,**right_up)
        # self.end_action()

    def DownAndUp(self):
        #sleep(2)
        point = self.kinematics.get_kinematics_pose('left_arm')
        point_down = point.copy()
        point_down["z"] = 0.82
        #print('point_down',point_down)
        #print('point',point)
        self.kinematics.set_kinematics_pose(group_name="left_arm",time=2,**point_down)
        self.end_action()
        self.kinematics.set_kinematics_pose(group_name="left_arm",time=3,**point)
        self.end_action()

    def finish(self):
        pass
    
    def grip_off(self,group):
        msg = JointState()
        msg.name = [group]
        msg.position = [0]
        msg.velocity = [0]
        msg.effort = [250]
        
        msg.header.frame_id = ''
        msg.header.stamp.secs = 0
        msg.header.stamp.nsecs= 0
        for i in range(4):
            msg.header.seq = i
            self.pub_gripper.publish(msg)
            self.rate.sleep()
            #print(i) 
        self.end_action()

    def grip_half(self,group):
        msg = JointState()
        msg.name = [group]
        msg.position = [0.6]
        msg.velocity = [0]
        msg.effort = [250]
        
        msg.header.frame_id = ''
        msg.header.stamp.secs = 0
        msg.header.stamp.nsecs= 0
        for i in range(4):
            msg.header.seq = i
            self.pub_gripper.publish(msg)
            self.rate.sleep()
            #print(i) 
        self.end_action()


    def grip_on(self,group):
        msg = JointState()
        msg.name = [group]
        msg.position = [1.15]
        msg.velocity = [0]
        msg.effort = [250]
        
        msg.header.frame_id = ''
        msg.header.stamp.secs = 0
        msg.header.stamp.nsecs= 0
        for i in range(4):
            msg.header.seq = i
            self.pub_gripper.publish(msg)
            self.rate.sleep()
            #print(i) 
        self.end_action()


if __name__ == '__main__':
    rospy.init_node('magic_main', anonymous=False)
    rospy.loginfo("Magic Main - Running")

    main = magic()
    main.run()
