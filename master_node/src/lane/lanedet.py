
#-*- coding:utf-8 -*-

'''
self.control_data 는 실시간으로 master.py, sender.py와 공유됨 
'''

# from master_node.msg import lane
# from std_msgs.msg import Time



import pymap3d
#import csv
import rospy
from nav_msgs.msg import Odometry
from nav_msgs.msg import Odometry
from std_msgs.msg import String
# from obstacle_detector.msg import Obs
import numpy as np
from math import radians
from math import degrees
from math import sin
from math import cos
from math import hypot
from math import atan2
from math import pi
from math import sqrt
# import LPP
import sys
import serial
import struct
import time
import matplotlib.pyplot as plt
# from hybrid_a_star import path_plan 
from darknet_ros_msgs.msg import BoundingBoxes
from geometry_msgs.msg import Pose, Point, Vector3, Quaternion
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from std_msgs.msg import Time
from master_node.msg import lane
# from master_node.msg import BoundingBoxes
from master_node.msg import BoundingBox
from visualization_msgs.msg import Marker
from std_msgs.msg import Float32




# sys.path.append("./map1/")
# encoder velocity
from std_msgs.msg import Int64

# try:
#     import makeTarget
# except:
#     raise

class Controller:
    def __init__(self,  master):

        self.control_data = master.control_data
        self.goal_x = 0
        self.goal_y = 0
        self.past_state = 'vision_lane'
        self.cur_state = 'vision_lane'
        self.vision_lane_data = lane()
        self.lidar_lane_data = lane()
        self.corn_flag = True
        self.ser = serial.Serial('/dev/ttyUSB2',115200) # USB 권한 주

        self.drive_mode = 'normal'


        self.left_line_pub = rospy.Publisher('left_line', Marker, queue_size = 10)
        self.right_line_pub = rospy.Publisher('right_line', Marker, queue_size = 10)


        # 종제어 관련 ############
        self.WB = 1.04
        self.target_index = 0
        self.speed = 0
        self.cnt = 0

        self.t_start = 0
        self.t_delta = 0
        self.t_old = 0
        self.t_new = 0
        self.t = 0


        self.Kp_v = 50
        self.Ki_v = 5
        self.Kd_v = 10
        '''
        self.Kp_s = 0.5
        self.Ki_s = 0.01
        self.Kd_s = 0''' # 얘는 무얼까아 _ 뭐쥐 @@@@@@@@@@
        self.V_veh = 0
        self.V_err = 0
        self.V_err_old = 0
        self.V_err_pro = 0
        self.V_err_inte = 0
        self.V_err_deri = 0

        self.safety_factor = 0.8
        self.V_ref_max = 10 #[km/h] ####@@@@@@ 정현아이게 비젼운행시 최고속도야 ( 높여줘. )
        self.curve_flag = False # 직선에서 출발 @@@@@@@2
        # encoder
        self.e = 0
        self.f = 0
        self.cur_data_right = 0
        self.displacement_right = 0
        self.dis_DR_enc_right = 0
        self.velocity = 0
        self.dis_DR_flag = 0
        self.velocity_enc = 0
        ######################\

        self.lane_flag = 1
        self.goal_x_buffer = 0
        self.goal_y_buffer = 0
        
        self.avoid_steer = False

        self.paths_right = PointCloud()
        # self.paths_right.header= Header()
        self.paths_right.header.frame_id = 'world'
        self.paths_right.header.stamp = rospy.Time.now()
        self.pub_pr = rospy.Publisher('/path_r', PointCloud, queue_size=1)

        self.paths_left = PointCloud()
    
        self.paths_left.header.frame_id = 'world'
        self.paths_left.header.stamp = rospy.Time.now()
        self.pub_pl = rospy.Publisher('/path_l', PointCloud, queue_size=1)

        self.goal_pt = PointCloud()
        self.goal_pt.header.frame_id = 'world'
        self.goal_pt.header.stamp = rospy.Time.now()
        self.pub_g = rospy.Publisher('/goal_p', PointCloud, queue_size=1)
        
        

        



    # def connect(self):
    #         self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) #UDP LAN
    #         recv_address = ('127.0.0.1', 3052)
    #         self.sock.bind(recv_address)

    #         print("Connect!")

    def line_marker(self, fir, last, name, idd):
        scale = Vector3(0.04, 0.04, 0.04)
        line_mk = Marker()
        line_mk.header.frame_id = "world"
        line_mk.header.stamp = rospy.Time.now()
        line_mk.ns = name
        line_mk.id = idd
        line_mk.type = Marker.ARROW
        line_mk.action = Marker.ADD
        line_mk.pose.position.x = 0.0
        line_mk.pose.position.y = 0.0
        line_mk.pose.position.z = -0.1
        line_mk.pose.orientation.x = 0.0
        line_mk.pose.orientation.y = 0.0
        line_mk.pose.orientation.z = 0.0
        line_mk.pose.orientation.w = 0.0
        line_mk.scale = scale
        line_mk.color.r = 1.0
        line_mk.color.g = 0.0
        line_mk.color.b = 0.0
        line_mk.color.a = 1.0
        line_mk.points.append(fir)
        line_mk.points.append(last)
        return line_mk

    def getcurstate(self, msg):
        temp_msg = msg.data
        if temp_msg == 'vision_start':
            self.cur_state = 'vision_lane'
        ## 지금 lidar 정보에서 curvature로 가장 최소 거리 들어오고 있는거 그거 추가해서 상황 판단 가능합니ㅏㄷ~~~
        elif temp_msg == 'lidar_start':
            self.cur_state = 'lidar_lane'
        elif temp_msg =='':
            print("pass")
        else:
            print("get state message Error!!")

    def getlidarlineline(self, msg):
        # self.lidar_lane_data = lane()
        # print("get lidar lane~~~")
        self.lidar_lane_data = msg
        if msg.curvature > 0.5:
            self.avoid_steer = False
        
        if self.curve_flag is True:
            if msg.curvature < 0.5:
                self.avoid_steer = True
        
        ##라이다에서 들어오는 정보 없으면 차선으로 돌리는 부분인데 확인 함 필요해 보임더~~~
        

    def getvisionline(self, msg):
        # print("do?")
        self.vision_lane_data = msg

    def getboundingbox(self, msg):
        temp = msg.bounding_boxes
        # print(len(temp))
        
        for i in range(len(temp)):
            if temp[i].Class == "CONE":
                self.cur_state = 'lidar_lane'
                print("dosdddddddddd")
        # print(temp[0].probability)
        # print("??", msg.bounding_boxes[0][0])

             
    def cal_steering(self, cur_x, cur_y, cur_yaw):
        # print(self.cur_state)
        if self.cur_state == 'vision_lane' : # 노협로
            # print("left_vi :", len(self.vision_lane_data.left))
            # print("right_vi :", len(self.vision_lane_data.right))
            self.getline(self.vision_lane_data)
            


        elif self.cur_state == 'lidar_lane': # 협로
            # print("left :", len(self.lidar_lane_data.left))
            # print("right :", len(self.lidar_lane_data.right))
            self.getline(self.lidar_lane_data)

        else:
            print("Error!! State Empty")

        if self.lane_flag == 1:
            return self.steering_angle(cur_x, cur_y, cur_yaw, self.goal_x, self.goal_y)
        else:
            # self.goal_x = self.goal_x_buffer
            # self.goal_y = self.goal_y_buffer
            return self.steering_angle(cur_x, cur_y, cur_yaw, self.goal_x, self.goal_y)

    def steering_angle(self,  cur_x, cur_y, cur_yaw, target_x, target_y):
        cur_yaw = 0
        # print(target_x, target_y)
        tmp_th = degrees(atan2((target_y - cur_y), (target_x - cur_x)))
        tmp_th = tmp_th%360
        alpha =  cur_yaw - tmp_th                   # 
        # print("alpha before : ",alpha)
        if abs(alpha)>180: # -pi ~ pi
            if (alpha < 0) :
                alpha += 360
            else :
                alpha -= 360
        # print("alpha mid : ",alpha)
        alpha = max(alpha,  -90)
        alpha = min(alpha,  90)
        # print("alpha final : ",alpha)
        print("x, y", target_x, target_y)
        
        #PUREPURSUIT

        distance = ((target_x - cur_x)**2 + (target_y - cur_y)**2)**(1/2)
        # print('distance:',distance)
        # distance = 5
        # print('distance : ', distance)
        
        delta = degrees(atan2(2*self.WB*sin(radians(alpha))/distance,1))   # 
        
        # 곡선일때는 alpha 가보자. @@@@@@@22
        if self.curve_flag ==True:
            delta = alpha  #  이게 문제가 아닐까아러아러아러ㅏㅇ러ㅏㅇㄹㅇ
        # print('curve_flag',self.curve_flag)
        # print("len : ", len(lane)
        print('alpha, delta [deg] : ',alpha,delta)


        
        if abs(delta)>180:
            if (delta < 0) :
                delta += 360
            else :
                delta -= 360
        # print("delta", delta)

        if self.avoid_steer is True:
            delta += 50

        if abs(delta)>30:
            if delta > 0:
                return 1999
            else :
                return -1999
        else :    
            delta = 71*delta
        # print("delta", delta)

        #와리가리 
        # if self.cur_state == "vision_lane"
        #     if abs(alpha)<10 and abs(delta)<10:
        #         delta = 0
        return int(delta)

    def line_detect(self, point):
        x_sum = 0
        y_sum = 0
        number = len(point)
        for i in range(number):
            x_sum += point[i].x
            y_sum += point[i].y
        number = float(number)
        x_mean = x_sum / number
        y_mean = y_sum / number

        up = 0
        down = 0

        for i in range(int(number)):
            up += (point[i].x-x_mean) * (point[i].y - y_mean)
            down += (point[i].x-x_mean) ** 2

        #####zero division except######
        if down ==0:
            down = 0.000000000000001
        ##############################
        a = up / down
        b = y_mean - a * x_mean

        return a, b

    def getline(self, lane_data):
        left_first = Point32()
        left_last = Point32()
        right_first = Point32()
        right_last = Point32()
        center_first = Point32()
        center_last = Point32()

        # print(lane_data)

        # left is not empty
        left_x=[]
        left_y=[]
        right_x=[]
        right_y=[]

        for i in range(len(lane_data.left)):
            left_x.append(lane_data.left[i].x)
            left_y.append(lane_data.left[i].y)

        for i in range(len(lane_data.right)):
            right_x.append(lane_data.right[i].x)
            right_y.append(lane_data.right[i].y)
        # print(left_x,right_x)
        # print(right_y,left_y)

        # if len(lane_data.left) != 0:
        #     l_a, l_b = self.line_detect(lane_data.left)
        #     left_first.x = min(left_x)
        #     left_first.y = min(left_x) * l_a + l_b
        #     left_last.x = max(left_x)
        #     left_last.y = max(left_x) * l_a + l_b # - 0.2
        #     # left_last.x = 2
        #     # left_last.y = 2 * l_a + l_b

        # else:
        #     l_a, l_b = 0, 0
        #     left_first.x = 0
        #     left_first.y = 0
        #     left_last.x = 0
        #     left_last.y = 0

        # right is not empty
        if len(lane_data.right) != 0:
            r_a, r_b = self.line_detect(lane_data.right)
            right_first.x = min(right_x)
            right_first.y = min(right_x) * r_a + r_b
            right_last.x = max(right_x)
            right_last.y = max(right_x) * r_a + r_b # -0.2
        else:
            r_a, r_b = 0, 0
            right_first.x = 0
            right_first.y = 0
            right_last.x = 0
            right_last.y = 0
            




        # c_a = (l_a + r_a) / 2
        # c_b = 0.0
        

        # center_first.x = 0.0
        # center_first.y = 0.0
        # center_last.x = 1.0
        # center_last.y = 1.0 * c_a + c_b
        
        # left_last.x, right_last.x = max(left_last.x, right_last.x), max(left_last.x, right_last.x)
        # left_last.y, right_last.y = left_last.x * l_a + l_b, right_last.x * r_a + r_b

        # left_last.x, left_last.y = max(left_last.x, right_last.x), left_last.x * l_a + l_b
        # , right_last.y = left_last.x * l_a + l_b, right_last.x * r_a + r_b

        # 디디디디디디
        d = 1.3
        # print("number :", len(lane_data.left), len(lane_data.right))
        # l_rad=np.arctan2(left_last.y - left_first.y, left_last.x - left_first.x) - pi / 2
        r_rad=np.arctan2(right_last.y - right_first.y, right_last.x - right_first.x) +pi / 2

        if len(lane_data.right)!= 0:
            # self.goal_x, self.goal_y = 1, l_a
            self.goal_x, self.goal_y = right_first.x + (d*cos(r_rad)), right_first.y + (d*sin(r_rad))
            # print(left_last.x, right_last.x, right_last.y, right_last.y)

        elif len(lane_data.right) == 0:
            self.goal_x, self.goal_y = 3, -1
            self.curve_flag =True
            
            # print(left_last.x, right_last.x, right_last.y, right_last.y)
        print("len : ", len(lane_data.right))
        # elif len(lane_data.left) == 0:
        #     self.goal_x, self.goal_y = 5, 1
            
        # elif len(lane_data.left) != 0 and len(lane_data.right) == 0:
        #     # print("left_deg : ",np.rad2deg(rad2))
        #     self.goal_x = left_last.x + (d*cos(l_rad))
        #     self.goal_y = left_last.y + (d*sin(l_rad))
        #     # self.goal_x, self.goal_y = 1, -1

        # elif len(lane_data.left) == 0 and len(lane_data.right) != 0:
        #     # print("right_deg : ",np.rad2deg(r_rad))
        #     self.goal_x = right_last.x + (d*cos(r_rad))
        #     self.goal_y = right_last.y + (d*sin(r_rad))
        #     # self.goal_x, self.goal_y = 1, 19

        r_line = self.line_marker(right_first, right_last, "left", 0)
        # r_line = self.line_marker(right_first, right_last, "right", 1)

        # if len(left_x) == 0 and len(right_x) == 0:
        #     self.serWrite(self.speed,int(self.control_data['steering']), self.cnt)
        self.right_line_pub.publish(r_line)
        
        if len(right_x) == 0: #both empty array
            self.lane_flag = 0

        else: 
            self.lane_flag = 1
            self.goal_x_buffer = self.goal_x
            self.goal_y_buffer = self.goal_y


        self.paths_right = PointCloud()
        self.paths_left = PointCloud()
        self.goal_pt = PointCloud()
        
        self.paths_right.header.frame_id = 'world'
        self.paths_right.header.stamp = rospy.Time.now()
        
        self.paths_left.header.frame_id = 'world'
        self.paths_left.header.stamp = rospy.Time.now()
        
        self.goal_pt.header.frame_id = 'world'
        self.goal_pt.header.stamp = rospy.Time.now()
        
        #시각화
        for i in range(len(right_x)):
            path = Point32()
            path.x = right_x[i]
            path.y = right_y[i]
            self.paths_right.points.append(path)

        # for i in range(len(right_x)):
        #     path = Point32()
        #     path.x = right_x[i]
        #     path.y = right_y[i]
        #     self.paths_right.points.append(path)
        
        goal = Point32()
        goal.x = self.goal_x
        goal.y = self.goal_y
        self.goal_pt.points.append(goal)

        # self.pub_pl.publish(self.paths_right)
        self.pub_pr.publish(self.paths_right)
        self.pub_g.publish(self.goal_pt)
        


    def serWrite(self,speed, steering, cnt):
        break_val = 0x01

        # if self.drive_mode == 'stop':
        #     V_in = 0x00
        #     break_val = 50
        
        #스피드 바꾸어 주는 부분이 아래 int(10) 되어있는 부분임다 빠르게 진행하시려면 숫자 키워주시면 됩니다링
        result = struct.pack('!BBBBBBHhBBBB', 0x53, 0x54, 0x58, 0x01, 0x00, 0x00, speed,
                    steering, break_val, self.cnt, 0x0D, 0x0A )    # big endian 방식으로 타입에 맞춰서 pack   
        self.ser.write(result)

    def getOdoMsg(self,  msg):
        # print(self.cur_state)
        if len(self.lidar_lane_data.left) == 0 and len(self.lidar_lane_data.right) == 0:
            self.cur_state = 'vision_lane'
        # print(self.cur_state)
        self.control_data['cur_yaw']  = 0
        self.control_data['cur_x'], self.control_data['cur_y']  = 0, 0



        packet = self.ser.readline()
        # print(packet) 
        if len(packet) == 18:
            header = packet[0:3].decode()

            if header == "STX":

                tmp1, tmp2 = struct.unpack("2h", packet[6:10])
                self.V_veh = tmp1   # *10 곱해진 값임.
                # print('V_veh:',self.V_veh)

                self.cnt = struct.unpack("B", packet[15:16])[0]

        
            # if len(self.vision_lane_data) != 0 or len(self.lidar_lane_data)!=0:


            self.calc_velocity_encoder()

            ###  seiral에 넣어줄  steering, speed 
            self.control_data['steering']= self.cal_steering(self.control_data['cur_x'], self.control_data['cur_y'], self.control_data['cur_yaw'])
            

            # print(self.control_data['steering'])
            # print(self.goal_x,self.goal_y)

            if self.cur_state == "lidar_lane":
                self.speed= int(45)    
            elif self.cur_state == "vision_lane":
                self.speed= self.calc_velocity()
                # self.speed= int(45)
                # 곡선일때는 alpha 가보자. @@@@@@@22
                if self.curve_flag ==True:
                    self.speed=int(40) 
                    self.curve_flag =False

                    print('speed:',self.speed)
                    # print('speed:',speed)
                

            # print("현재 속도", self.velocity_enc)
            # self.cnt=0
            self.serWrite(self.speed,int(self.control_data['steering']), self.cnt)
            self.past_state = self.cur_state # 바로전 모드 기록 _ 종제어 용







    ##### 종제어 speed 반환.3종 set _V_ref 에 따른 V_in #############

    def calc_velocity(self): # 최적의 V_in 을 return 어디에 넣을까. GPP 상태에서 받아오도록 . getodo 에 넣으면 될듯 .. 
        if self.past_state != 'vision_lane' and self.cur_state=='vision_lane' : #  lidar 에서 vison 갈때
            self.t_start = time.time()
            self.t_new = 0
            self.t_delta = 0
            self.t_old = 0
            self.t_new = 0
            self.t = 0

            self.V_err = 0
            self.V_err_old = 0
            self.V_err_inte = 0
            self.V_err_deri = 0
            self.V_err_pro = 0

        V_ref = self.calc_Vref() # 10 곱해서 준 값. 
        
        # @@@@ 양쪽 곡률 있을 때.
        # if V_ref != (self.V_ref_max)*10:
        #     self.curve_flag =True
        # else:
        #     self.curve_flag =False


        V_in = self.PID(V_ref)


        if V_in > 200:
            V_in = 200
        elif V_in < V_ref:
            V_in = V_ref
        print('V_ref, V_in, V_veh:',V_ref,V_in,self.velocity_enc*36) # 10 곱해진채로 뜰거.
        
        
        return int(V_in)

    def PID(self,V_ref):
        
        # self.V_err = V_ref - self.V_veh #cccccccccc
        self.V_err = V_ref - self.velocity_enc*3.6*10 # 10 곱해진 상태.
        

        self.t_old = self.t_new
        self.t_new = time.time()
        self.t_delta = self.t_new - self.t_old
        self.t = time.time() - self.t_start

        self.V_err_pro = self.Kp_v * self.V_err
        if self.Ki_v * self.V_err * self.t_delta < 100:
            self.V_err_inte += self.Ki_v * self.V_err * self.t_delta
        self.V_err_deri = self.Kd_v * (self.V_err - self.V_err_old) / self.t_delta

        V_in = self.V_err_pro # + self.V_err_inte + self.V_err_deri
        self.V_err_old = self.V_err

        return V_in

    def calc_Vref(self):
        # self.stidx = self.select_target(self.control_data['cur_x'], self.control_data['cur_y'], self.stidx, self.path_x, self.path_y, self.control_data['speed_look_ahead'])

        target_k = self.vision_lane_data.curvature # vision lanedetection publish 할때 받음 > 주파수 확인.
        # print('curvature:',self.vision_lane_data.curvature) 
        critical_k = ((self.safety_factor/self.V_ref_max)**2) * 19.071
        
        if target_k < critical_k:
            V_ref = self.V_ref_max
        else:
            V_ref = self.safety_factor * (sqrt(19.071/target_k)) # 이거 테스트해보고 아니면 그냥 뭐 속도 '절반' 이렇게 박아도 될듯. 

        if V_ref < 3: # 50 으로 minimum limit 걸자. 너무 느린듯.  speed_lookahead 도 좀 줄이자 3 정도로 dddddddd
            V_ref= 3  # ddddddddd 

        
        return 10 * float(V_ref)

    def calc_velocity_encoder(self):
        self.displacement_right = self.cur_data_right

        if self.dis_DR_flag == 0 :
            self.e = self.displacement_right
            self.f = self.displacement_right
            self.dis_DR_flag = 1

        elif self.dis_DR_flag != 0 :
            self.e = self.f
            self.f = self.displacement_right

        if ((self.f - self.e) < -10000000):
            self.dis_DR_enc_right = (self.f + (256**4 - self.e))*1.6564/100 
        else:
            self.dis_DR_enc_right = (self.f - self.e)*1.6564/100

        self.velocity_enc = self.dis_DR_enc_right / 0.1

    def Get_Dis_right(self, data):
        res = data.data
        # print(res)
        self.cur_data_right = int(res)









    def run(self):
        # rospy.init_node("vision_lidar", anonymous=True)
        # self.connect()
        rospy.Subscriber("/Displacement_right", Int64, self.Get_Dis_right)
        rospy.Subscriber("/timer", Time, self.getOdoMsg)
        rospy.Subscriber("/lidar_pub", lane, self.getlidarlineline)
        rospy.Subscriber("/laneinformation", lane, self.getvisionline)
        rospy.Subscriber("/laneinformation1", lane, self.getvisionline)

        rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, self.getboundingbox)
        

