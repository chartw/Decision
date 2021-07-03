#-*- coding:utf-8 -*-

'''
self.control_data 는 실시간으로 master.py, sender.py와 공유됨 
'''

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
# import LPP
import sys
import serial
import struct
import matplotlib.pyplot as plt
from hybrid_a_star import path_plan 
from master_node.msg import Obstacles 
from master_node.msg import PangPang
from darknet_ros_msgs.msg import BoundingBoxes
from geometry_msgs.msg import Pose, Point, Vector3, Quaternion
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from std_msgs.msg import Time
sys.path.append("./map1/")

try:
    import makeTarget
except:
    raise

class Controller:
    def __init__(self,  master):
        self.control_data = master.control_data
        self.path_x = []
        self.path_y = []
        self.path_yaw = []
        self.lpath_x = []
        self.lpath_y = []
        self.goal_x = 0
        self.goal_y = 0
        self.final_switch = 0

        self.ser = serial.Serial('/dev/ttyUSB0',115200) # USB 권한 주
        
    def get_xy(self,  lat,  lon,  alt):
        e, n, u = pymap3d.geodetic2enu(lat, lon, alt, self.control_data['base_lat'],  self.control_data['base_lon'],  self.control_data['base_alt'])
        return e,  n
             
    def cal_steering(self, cur_x, cur_y, cur_yaw):
        return self.steering_angle(cur_x, cur_y, cur_yaw, self.goal_x, self.goal_y)

    def steering_angle(self,  cur_x, cur_y, cur_yaw, target_x, target_y):
        cur_yaw = 0
        tmp_th = degrees(atan2((target_y - cur_y), (target_x - cur_x)))
        tmp_th = tmp_th%360
        # print("temp", tmp_th)
        alpha =  cur_yaw - tmp_th
        # alpha = 360 - alpha
        # print("what?: ", alpha)
        if abs(alpha)>180:
            if (alpha < 0) :
                alpha += 360
            else :
                alpha -= 360
        # print("what2?: ", alpha)
        alpha = max(alpha,  -90)
        alpha = min(alpha,  90)
        # print("alpha", alpha)
        # delta = degrees(atan2(2*self.control_data['WB']*sin(radians(alpha))/1,1))
        # print("delta :", delta)
        delta = alpha
        # if target_y>0:
        #     delta = -alpha
        # print(delta)
        if abs(delta)>180:
            if (delta < 0) :
                delta += 360
            else :
                delta -= 360
        print("delta", delta)
        if abs(delta)>30:
            if delta > 0:
                return 1999
            else :
                return -1999
        else :
            delta = 71*delta

        
        return int(delta)

    def serWrite(self, speed, steering, cnt):
        input_speed = speed
        break_val = 0x01
        # steering 값 2000 넘길 시 2000으로 설정
        if abs(steering)>=2000:
            if steering>0:
                steering = 1999
            else :
                steering =-1999
        # if abs(steering) < 300 :
        #     steering = int(0.5*steering)

        # 기어 기본값 0: 전진, 1:후진
        goal_dis = hypot(self.goal_x - self.control_data['cur_x'], self.goal_y - self.control_data['cur_y']) # 하이팟 대신에 x, y 따로 비교를 할까...

        # if goal_dis > 3:
        #     speed = speed
        # else:
        #     break_val = 0x0B #
        #     speed = 0x00

        # if self.person_detect is 1:
        #     break_val = 150

        result = struct.pack('!BBBBBBHhBBBB', 0x53, 0x54, 0x58, 0x01, 0x00, 0x00, int(speed),
                    steering, break_val, cnt, 0x0D, 0x0A )    # big endian 방식으로 타입에 맞춰서 pack   
        
        self.ser.write(result)


    def calc_dis(self, nx, ny):
        
        distance = ((nx - self.control_data['cur_x'])**2 +  (ny - self.control_data['cur_y'])**2)**0.5

        return distance

    def final(self):
        self.control_data['first_check'] = False
        
    def select_goal(self, c_x, c_y, c_yaw, ox, oy):

        gidx = 0
        valid_idx = []
        px, py = self.path_x, self.path_y
        px_rot, py_rot = [], []

        for i in range(self.control_data['target_idx'], self.control_data['target_idx']+300):
            valid_idx.append(i)

        for i in range(len(valid_idx)):
            px[valid_idx[i]] -= c_x
            py[valid_idx[i]] -= c_y
            
        rotate_yaw = c_yaw-90 % 360
        for i in range(len(valid_idx)):
            px_rot.append(px[valid_idx[i]]*cos(np.deg2rad(rotate_yaw)) + py[valid_idx[i]]*sin(np.deg2rad(rotate_yaw)))
            py_rot.append(px[valid_idx[i]]*-sin(np.deg2rad(rotate_yaw)) + py[valid_idx[i]]*cos(np.deg2rad(rotate_yaw)))
        print(len(valid_idx), len(px_rot))
        for i in range(len(valid_idx)):
            px_rot[i] += c_x
            py_rot[i] += c_y

        for i in range(len(valid_idx)):
            if px_rot[i] < max(ox) and py_rot[i] < max(oy):
                if px_rot[i] > min(ox) and py_rot[i] > min(oy):
                    gidx = valid_idx[i]-10

        return gidx
        


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

        a = up / down
        b = y_mean - a * x_mean

        return a, b

    def getline(self, msg):
        left = []
        right = []
        center = []
        left_temp = PointCloud()
        right_temp = PointCloud()
        center_temp = PointCloud()
        left_first = Point32()
        left_last = Point32()
        right_first = Point32()
        right_last = Point32()
        center_first = Point32()
        center_last = Point32()
        left_x = []
        right_x = []
        for i in range(len(msg.points)):
            if msg.points[i].y > 0 :
                left = msg.points[i]
                left_x.append(msg.points[i].x)
                left_temp.points.append(left)
            else:
                right = msg.points[i]
                right_x.append(msg.points[i].x)
                right_temp.points.append(right)

        if len(left_temp.points) != 0:
            l_a, l_b = self.line_detect(left_temp.points)
            left_first.x = min(left_x)
            left_first.y = min(left_x) * l_a + l_b
            left_last.x = max(left_x)
            left_last.y = max(left_x) * l_a + l_b
        else:
            l_a, l_b = 0, 0
            left_first.x = 0
            left_first.y = 0
            left_last.x = 0
            left_last.y = 0
        if len(right_temp.points) != 0:
            r_a, r_b = self.line_detect(right_temp.points)
            right_first.x = min(right_x)
            right_first.y = min(right_x) * r_a + r_b
            right_last.x = max(right_x)
            right_last.y = max(right_x) * r_a + r_b
        else:
            r_a, r_b = 0, 0
            right_first.x = 0
            right_first.y = 0
            right_last.x = 0
            right_last.y = 0


        c_a = (l_a + r_a) / 2
        c_b = 0.0
        # left_first.x = left_temp.points[0].x
        # left_first.y = left_temp.points[0].x * l_a + l_b
        # left_last.x = left_temp.points[len(left_temp.points)-1].x
        # left_last.y = left_temp.points[len(left_temp.points)-1].x * l_a + l_b

        # right_first.x = right_temp.points[0].x
        # right_first.y = right_temp.points[0].x * r_a + r_b
        # right_last.x = right_temp.points[len(right_temp.points)-1].x
        # right_last.y = right_temp.points[len(right_temp.points)-1].x * r_a + r_b



        

        center_first.x = 0.0
        center_first.y = 0.0
        center_last.x = 2.0
        center_last.y = 2.0 * c_a + c_b
        d=1.2
        l_rad=np.arctan2(left_last.y - left_first.y, left_last.x - left_first.x)
        r_rad=np.arctan2(right_last.y - right_first.y, right_last.x - right_first.x) +pi/2
        if len(left_temp.points)!= 0 and len(right_temp.points) != 0:
            self.goal_x, self.goal_y = (left_last.x + right_last.x)/2 , (left_last.y + right_last.y)/2
        elif len(left_temp.points)== 0 and len(right_temp.points) == 0:
            self.goal_x, self.goal_y = 1, 0
        elif len(left_temp.points) != 0 and len(right_temp.points) == 0:
            print("left_deg : ",np.rad2deg(rad2))
            self.goal_x = left_last.x + (d*cos(l_rad))
            self.goal_y = left_last.y + (d*sin(l_rad))
        elif len(left_temp.points) == 0 and len(right_temp.points) != 0:
            print("right_deg : ",np.rad2deg(r_rad))

            self.goal_x = right_last.x + (d*cos(r_rad))
            self.goal_y = right_last.y + (d*sin(r_rad))   

        # if len(left_temp.points)!= 0 and len(right_temp.points) != 0:
        #     self.goal_x, self.goal_y = (left_last.x + right_last.x)/2 , (left_last.y + right_last.y)/2
        # elif len(left_temp.points)== 0 and len(right_temp.points) == 0:
        #     self.goal_x, self.goal_y = 1, 0
        # elif len(left_temp.points) != 0 and len(right_temp.points) == 0:
        #     self.goal_x, self.goal_y = 1, l_a
        # elif len(left_temp.points) == 0 and len(right_temp.points) != 0:
        #     self.goal_x, self.goal_y = 1, r_a   

        # if len(left_temp.points)!= 0 and len(right_temp.points) != 0:
        #     self.goal_x, self.goal_y = (left_last.x + right_last.x)/2 , (left_last.y + right_last.y)/2
        # elif len(left_temp.points)== 0 and len(right_temp.points) == 0:
        #     self.goal_x, self.goal_y = 1, 0
        # elif len(left_temp.points) != 0 and len(right_temp.points) == 0:
        #     self.goal_x, self.goal_y = 1, -1
        # elif len(left_temp.points) == 0 and len(right_temp.points) != 0:
        #     self.goal_x, self.goal_y = 1, 1   
        # print("x, y : ", self.goal_x, self.goal_y)
        # print(self.control_data['cur_x'], self.control_data['cur_y'])


    def getOdoMsg(self,  msg):
        if self.goal_x is not 0:
            # cur_lon    = msg.pose.pose.position.x
            # cur_lat    = msg.pose.pose.position.y
            #print('cur_lon:', cur_lon, 'cur_lat:', cur_lat)
            self.control_data['cur_yaw']  = 0
            # self.control_data['cur_yaw']  = msg.twist.twist.angular.z #imu 정북 기준으로 시계로 돌아갈때(시뮬)
            # self.control_data['cur_yaw'] = 360 - msg.twist.twist.angular.z #imu 정북 기준으로 시계로 돌아갈때(송도)
        
            # self.control_data['cur_x'], self.control_data['cur_y']  = self.get_xy(cur_lat,  cur_lon,  self.control_data['base_alt'])
            self.control_data['cur_x'], self.control_data['cur_y']  = 0, 0
            self.control_data['steering']  = self.cal_steering(self.control_data['cur_x'], self.control_data['cur_y'],  self.control_data['cur_yaw'])
            cnt=0x00
            
            result = self.ser.readline()
            # print(result)
            self.ser.flushInput()

            if (result[0] is 0x53 and result[1] is 0x54 and result[2] is 0x58):
                res_arr = []
                res_idx = 0

                while True:
                    for i in range(len(result)):
                        if result[i] is 0x0A and i is not 17:
                            print("### 0x0A Found!", i, "th data")
                            res_arr.append(0x0B)
                        else :
                            res_arr.append(result[i])

                    if len(res_arr) < 18:
                        result = self.ser.readline()
                    else:
                        break

                cnt = res_arr[15]
                # print("res_arr is", res_arr)
                # print("do?")
                self.serWrite(int(self.control_data['target_speed']), int(self.control_data['steering']), cnt)
            if self.final_switch == 0:
                self.final()
                self.final_switch = 1

    def run(self):
        rospy.Subscriber("lidar_pub", PointCloud, self.getline)
        rospy.Subscriber("/timer", Time, self.getOdoMsg)


