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
import time
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
# from hybrid_a_star import path_plan 
from master_node.msg import Obstacles 
from master_node.msg import PangPang
from darknet_ros_msgs.msg import BoundingBoxes
from geometry_msgs.msg import Pose, Point, Vector3, Quaternion
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from std_msgs.msg import Time
from lane_detection.msg import lane
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

        self.drive_mode = 'normal'

        self.Kp_ld = 0.03

        self.Kp_v = 50
        self.Ki_v = 5
        self.Kd_v = 10

        self.Kp_s = 0.5
        self.Ki_s = 0.01
        self.Kd_s = 0

        self.lane_curv = 0
        self.lane_width=0

        self.curvature = 0
        self.speed_look_ahead = 6
        self.V_err = 0
        self.V_err_old = 0
        self.V_in = 0
        self.V_veh = 0
        self.V_err_pro = 0
        self.V_err_inte = 0
        self.V_err_deri = 0

        self.steering_veh = 0
        self.steering_err =0
        self.steering_err_old = 0
        self.steering_in = 0
        self.steering_err_inte = 0
        self.steering_err_deri = 0
        self.t_old =0 
        self.t_new=0
        self.t_delta=0
        self.t_start=0
        self.t=0

        
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
        delta = alpha
        # print("delta :", delta)
        # if target_y>0:
        #     delta = -alpha
        # print(delta)
        if abs(delta)>180:
            if (delta < 0) :
                delta += 360
            else :
                delta -= 360

        if abs(delta)>30:
            if delta > 0:
                # print('del;',  '1900')
                return 1999
            else :
                # print('del;',  '-1900' )
                return -1999
        else :
            delta = 71*delta
            # print('del:',  delta)
            return int(delta)

    def calc_velocity(self, k):
        V_ref_max = 15 # km/h
        n = 0.8 #safety factor

        critical_k = ((n/V_ref_max)**2) * 19.071

        if k < critical_k:
            V_ref = V_ref_max
        else:
            V_ref = n * (sqrt(19.071/k))

        return 10 * V_ref # 10*(km/h)

    def serWrite(self, V_in, V_veh, steering, cnt):

        # print(steering)
        # print(V_in)
        break_val = 0x00

        if self.drive_mode == 'stop':
            V_in = 0x00
            if V_veh > 150:
                break_val = 200
            elif V_veh > 100:
                break_val = 145
            elif V_veh > 50:
                break_val = 70
            else:
                break_val = 50

        result = struct.pack('!BBBBBBHhBBBB', 0x53, 0x54, 0x58, 0x01, 0x00, 0x00, int(V_in),
                    steering, break_val, cnt, 0x0D, 0x0A )    # big endian 방식으로 타입에 맞춰서 pack   
        # print("pc : ", result[0], result[1], result[2], result[3], result[4], result[5], result[6], result[7], result[8],
        # result[9], result[10], result[11], result[12], result[13] )
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
        self.lane_curv = msg.curvature

        left_first = Point32()
        left_last = Point32()
        right_first = Point32()
        right_last = Point32()
        center_first = Point32()
        center_last = Point32()

        left_x=[]
        left_y=[]
        right_x=[]
        right_y=[]

        for i in range(len(msg.left)):
            left_x.append(msg.left[i].x)
            left_y.append(msg.left[i].y)

        for i in range(len(msg.right)):
            right_x.append(msg.right[i].x)
            right_y.append(msg.right[i].y)

        if len(msg.left) != 0:
            l_a, l_b = self.line_detect(msg.left)
            left_first.x = min(left_x)
            left_first.y = min(left_y) * l_a + l_b
            left_last.x = max(left_x)
            left_last.y = max(left_y) * l_a + l_b
        else:
            l_a, l_b = 0, 0
            left_first.x = 0
            left_first.y = 0
            left_last.x = 0
            left_last.y = 0

        # right is not empty
        if len(msg.right) != 0:
            r_a, r_b = self.line_detect(msg.right)
            right_first.x = min(right_x)
            right_first.y = min(right_y) * r_a + r_b
            right_last.x = max(right_x)
            right_last.y = max(right_y) * r_a + r_b
        else:
            r_a, r_b = 0, 0
            right_first.x = 0
            right_first.y = 0
            right_last.x = 0
            right_last.y = 0


        c_a = (l_a + r_a) / 2
        c_b = 0.0
        

        center_first.x = 0.0
        center_first.y = 0.0
        center_last.x = 2.0
        center_last.y = 2.0 * c_a + c_b
        l_rad=np.arctan2(left_last.y - left_first.y, left_last.x - left_first.x) - pi / 2
        r_rad=np.arctan2(right_last.y - right_first.y, right_last.x - right_first.x) +pi / 2
        if len(msg.left)!= 0 and len(msg.right) != 0:
            self.lane_width=abs(left_first.y - right_first.y)
            self.goal_x, self.goal_y = (left_last.x + right_last.x)/2 , (left_last.y + right_last.y)/2
        elif len(msg.left)== 0 and len(msg.right) == 0:
            self.goal_x, self.goal_y = 1, 0
        elif len(msg.left) != 0 and len(msg.right) == 0:
            # self.goal_x = left_last.x + (self.lane_width/2*cos(l_rad))
            # self.goal_y = left_last.y + (self.lane_width/2*sin(l_rad))
            self.goal_x = left_last.x + (3/2*cos(l_rad))
            self.goal_y = left_last.y + (3/2*sin(l_rad))

        elif len(msg.left) == 0 and len(msg.right) != 0:

            # self.goal_x = right_last.x + (self.lane_width/2*cos(r_rad))
            # self.goal_y = right_last.y + (self.lane_width/2*sin(r_rad))
            self.goal_x = right_last.x + (3/2*cos(r_rad))
            self.goal_y = right_last.y + (3/2*sin(r_rad))

        print(self.goal_x,self.goal_y)



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
                            # print("### 0x0A Found!", i, "th data")
                            res_arr.append(0x0B)
                        else :
                            res_arr.append(result[i])

                    if len(res_arr) < 18:
                        result = self.ser.readline()
                    else:
                        break

                cnt = int(res_arr[15])
                self.V_veh = int(res_arr[6])
                # print(self.V_veh)
                self.steering_veh = int(res_arr[8]) 

                self.control_data['steering'] = self.cal_steering(self.control_data['cur_x'], self.control_data['cur_y'],  self.control_data['cur_yaw'])
                # self.control_data['target_speed'] = self.calc_velocity(self.lane_curv)
                self.control_data['target_speed'] = 30
                self.V_err_old = self.V_err
                self.V_err = self.control_data['target_speed'] - self.V_veh
                # print('V_ref : ', self.control_data['target_speed'])
                # print('V_veh : ', self.V_veh)
                # print('V_err : ', self.V_err)

                #V_ref, V_veh PID제어도 해볼예정(getOdoMsg 주기가 일정한지 확인 필요)
                self.t_old = self.t_new
                self.t_new = time.time()
                self.t_delta = self.t_new - self.t_old
                self.t = time.time() - self.t_start

                self.V_err_pro = self.Kp_v * self.V_err
                if self.Ki_v * self.V_err * self.t_delta < 100:
                    self.V_err_inte += self.Ki_v * self.V_err * self.t_delta
                self.V_err_deri = self.Kd_v * (self.V_err - self.V_err_old) / self.t_delta

                self.V_in = self.V_err_pro + self.V_err_deri + self.V_err_inte


                # print(self.V_in)
                if self.steering_in > 2000:
                    self.steering_in = 1999
                elif self.steering_in < -2000:
                    self.steering_in = -1999


                if self.V_in > 200:
                    self.V_in = 200
                if self. V_in < 0:
                    self.V_in = 0

                if self.V_in < self.control_data['target_speed']:
                    self.V_in = self.control_data['target_speed']

                self.serWrite(self.control_data['target_speed'], self.V_veh, int(self.control_data['steering']), cnt)


            if self.final_switch == 0:
                self.final()
                self.final_switch = 1

    def run(self):
        self.t_start=time.time()
        rospy.Subscriber("/laneinformation", lane, self.getline)
        rospy.Subscriber("/timer", Time, self.getOdoMsg)