#-*- coding:utf-8 -*-

'''
self.control_data 는 실시간으로 master.py, sender.py와 공유됨 
'''

import time
import pymap3d
#import csv
import rospy
# from master_node.msg import Local
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from std_msgs.msg import Float32
# from obstacle_detector.msg import Obs
import numpy as np
from math import radians
from math import degrees
from math import sin
from math import hypot
from math import atan2
from math import sqrt
# import LPP
import sys
import serial
import struct
import matplotlib.pyplot as plt
# from hybrid_a_star import path_plan 
# from gongteo_lane_save import cubic


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
        self.path_k = []
        self.goal_x = 0
        self.goal_y = 0
        self.final_switch = 0
        self.start_idx = 0
        self.ser = serial.Serial('/dev/ttyUSB0',115200) # USB 권한 주
        self.ls = 0

        self.drive_mode = 'normal'

        self.Kp_ld = 0.03

        self.Kp_v = 50
        self.Ki_v = 5
        self.Kd_v = 10

        self.Kp_s = 0.5
        self.Ki_s = 0.01
        self.Kd_s = 0

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

        self.cur_points = PointCloud()
        self.paths = PointCloud()
        #cur_x,y를 Pointcloud에 담아 publish
        self.pub_c = rospy.Publisher('/cur_xy', PointCloud, queue_size=1)
        self.pub_p = rospy.Publisher('/path', PointCloud, queue_size=1)
        self.pub_dis = rospy.Publisher('/Displacement', Float32, queue_size=1)
        self.paths.header.frame_id = 'world'
        self.paths.header.stamp = rospy.Time.now()
        self.cur_points.header.frame_id = 'world'
        self.cur_points.header.stamp = rospy.Time.now()
        self.msg = Float32()
        self.pre_add = 0 #엔코더에서 사용하는 저장변수
        self.now_add = 0 #엔코더에서 사용하는 저장변수
        self.enc_flag = 0 #엔코더에서 사용하는 flag
        

        self.t_start = 0
        self.t_delta = 0
        self.t_old = 0
        self.t_new = 0
        self.t = 0

        self.lat_err_array = []


        # self.bring_tmp_map()
    def get_xy(self,  lat,  lon,  alt):
        e, n, u = pymap3d.geodetic2enu(lat, lon, alt, self.control_data['base_lat'],  self.control_data['base_lon'],  self.control_data['base_alt'])
        return e,  n
    

    def select_target(self, cx, cy, cidx):
        valid_idx_list = []

        for i in range(cidx, len(self.path_x)):
            dis = ((self.path_x[i]-cx)**2 +(self.path_y[i]-cy)**2)**0.5
            if dis <= self.control_data['look_ahead']:
                valid_idx_list.append(i)
            if len(valid_idx_list) != 0 and dis > self.control_data['look_ahead']:
                break
        if len(valid_idx_list) == 0:
            return 0
        else:
            return valid_idx_list[len(valid_idx_list) - 1]

    def select_target_s(self, cx, cy, cidx):
        valid_idx_list = []

        for i in range(cidx, len(self.path_x)):
            dis = ((self.path_x[i]-cx)**2 +(self.path_y[i]-cy)**2)**0.5
            if dis <= self.speed_look_ahead:
                valid_idx_list.append(i)
            if len(valid_idx_list) != 0 and dis > self.speed_look_ahead:
                break
        if len(valid_idx_list) == 0:
            return 0
        else:
            return valid_idx_list[len(valid_idx_list) - 1]

    def cal_steering(self, cur_x, cur_y, cur_yaw, look_ahead):

        tidx = self.select_target(cur_x, cur_y, self.control_data['target_idx'])
        stidx = self.select_target_s(cur_x, cur_y, self.control_data['target_idx'])
        # print(tidx,stidx)
        
        target_x = self.path_x[tidx]
        target_y = self.path_y[tidx]
        target_k = abs(self.path_k[stidx])

        return self.steering_angle(cur_x, cur_y, cur_yaw, target_x, target_y, target_k)
             
    def steering_angle(self,  cur_x, cur_y, cur_yaw, target_x, target_y, target_k):
        # pure pursuit 계산되는 부분 
        tmp_th = degrees(atan2((target_y - cur_y), (target_x - cur_x))) # Angle between lookahead and x-axis
        
        # print('cur_x:',  cur_x ,  'cur_y',  cur_y,  'target_x:',  target_x,  'target_y:',  target_y,  'atan2((target_y - cur_y), (target_x - cur_x):',  tmp_th)

        tmp_th = tmp_th%360

        # if (tmp_th > 90) and (tmp_th <= 360):
        #     tmp_th -= 90
        # else:
        #     tmp_th += 270
        alpha =  cur_yaw - tmp_th  
        if abs(alpha)>180:
            if (alpha < 0) :
                alpha += 360
            else :
                alpha -= 360
        alpha = max(alpha,  -90)
        alpha = min(alpha,  90)
        delta = degrees(atan2(2*self.control_data['WB']*sin(radians(alpha))/self.control_data['look_ahead'],1))
        # print('delta:',  delta)
        
        if abs(delta)>180:
            if (delta < 0) :
                delta += 360
            else :
                delta -= 360
        
        
        # print("delta2 : ", delta)
        if abs(delta)>30:
            if delta > 0:
                # print('del;',  '1900')
                return 1999, target_k
            else :
                # print('del;',  '-1900' )
                return -1999, target_k
        else :
            delta = 71*delta
            # print('del:',  delta)
            return int(delta), target_k

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
        break_val = 0x01
        
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

    def select_start_node(self, cx, cy, cyaw):
        nodelist = []
        nodelist = makeTarget.node_set()
        min_dis = 99999
        min_idx = 10000
        temp_idx = 10000
        temp_dis = 9999

        for node in nodelist:
            temp_dis = self.calc_dis(nodelist[node].x, nodelist[node].y)
            if temp_dis < min_dis:
                min_dis = temp_dis
                min_idx = node



        return min_idx

    def final(self):
        self.control_data['target_idx'] = 0
        self.path_x, self.path_y = [], []
        # print(self.control_data['cur_x'])
        # print(self.control_data['cur_y'])
        self.start_idx = self.select_start_node(self.control_data['cur_x'], self.control_data['cur_y'], self.control_data['cur_yaw'])
        # print("### start_idx: ", self.start_idx)
        
        self.path_x, self.path_y, self.path_k = makeTarget.path_connect(str(self.start_idx), '24') # 출발노드, 도착노드 
        # self.path_x, self.path_y, self.path_k = makeTarget.path_connect('01','02') # 출발노드, 도착노드 
        # cubic("vel",self.path_x, self.path_y)
        # print(self.path_x)
        self.goal_x, self.goal_y = self.path_x[-1], self.path_y[-1]
        self.control_data['first_check'] = False


        for i in range(len(self.path_x)):
            path = Point32()
            path.x = self.path_x[i]
            path.y = self.path_y[i]
            self.paths.points.append(path)
               
    def start_LPP(self):
        g_x, g_y, g_yaw = 22.9, 43.8, 329 # 의미없음 나중에 지우자...

        if self.ls is 0 and abs(self.control_data['cur_x']-0) < 2 and abs(self.control_data['cur_y']- 0) < 2 :
            print("LPP mode start###")
            self.ls = 1
            
        if self.ls is 1:
            self.path_x, self.path_y = path_plan(self.control_data['cur_x'], self.control_data['cur_y'], self.control_data['cur_yaw'], g_x, g_y, g_yaw)

            if abs(self.control_data['cur_x']-8) < 1 and abs(self.control_data['cur_y']- 15) < 1:
                print("LPP mode over###")
                self.ls = 0
                self.final()     

    def calc_lat_err(self):
        min_lat_err = 999
        min_idx = 0
        for i in range(len(self.path_x)):
            lat_err = hypot(self.path_x[i] - self.control_data['cur_x'], self.path_y[i] - self.control_data['cur_y'])
            if lat_err < min_lat_err:
                min_lat_err = lat_err
                min_idx = i
                if i == len(self.path_x): # avoid out of range error
                    min_idx -= 1
            
        A = [self.path_x[min_idx] - self.control_data['cur_x'], self.path_y[min_idx] - self.control_data['cur_y']]
        B = [self.path_x[min_idx+1] - self.path_x[min_idx], self.path_y[min_idx+1] - self.path_y[min_idx]]

        v_out = A[0] * B[1] - A[1] * B[0]
        if v_out < 0 : 
            min_lat_err = - min_lat_err
        
        #
        #
        # print(min_lat_err, ",", self.t)
        #
        #

        self.lat_err_array.append(min_lat_err)
        
    def getOdoMsg(self,  msg):
        # print('a')
        # calculate position
        self.control_data['cur_x']    = msg.pose.pose.position.x
        self.control_data['cur_y']    = msg.pose.pose.position.y
        # self.control_data['cur_x'], self.control_data['cur_y']  = self.get_xy(cur_lat,  cur_lon,  self.control_data['base_alt'])

        if self.final_switch == 0:
            self.final()
            self.final_switch = 1

        cur_point = Point32()
        cur_point.x = self.control_data['cur_x']
        cur_point.y = self.control_data['cur_y']
        self.cur_points.points.append(cur_point)

        self.pub_c.publish(self.cur_points)
        self.pub_p.publish(self.paths)
        # print(len(self.cur_points.points))


        # yaw (0~360 deg, east)
        self.control_data['cur_yaw']  =  msg.twist.twist.angular.z
        # print(self.control_data['cur_yaw'])
        # print( ' x:',  self.control_data['cur_x'],'\n','y:',self.control_data['cur_y'],'\n','yaw:',  self.control_data['cur_yaw'])
        if self.control_data['first_check'] is False:
            cnt=0x00        
            result = self.ser.readline()
            # print(result)
            self.ser.flushInput()
            # print('1')
            if (ord(result[0]) is 0x53 and ord(result[1]) is 0x54 and ord(result[2]) is 0x58):
                res_arr = []
                res_idx = 0
                # print('1')

                while True:
                    for i in range(len(result)):
                        if result[i] is 0x0A and i is not 17:
                            res_arr.append(0x0B)
                        else :
                            res_arr.append(result[i])

                    if len(res_arr) < 18:
                        result = self.ser.readline()
                    else:
                        break

                cnt = int(ord(res_arr[15]))
                self.V_veh = int(ord(res_arr[6]))
                # print(self.V_veh)
                self.steering_veh = int(ord(res_arr[8])) 
                # print(self.V_veh)
                
                ###################################hanbin###################################
                # #difference 500넘을때 이전 encoder차이값만큼씩 더할 수 있도록 만들기.
                # #initial
                # self.pre_add = 0 #엔코더에서 사용하는 저장변수
                # self.now_add = 0 #엔코더에서 사용하는 저장변수
                # self.enc_flag = 0 #엔코더에서 사용하는 flag
                ############################################################################
                # a = ord(res_arr[11])
                # b = ord(res_arr[12])
                # c = ord(res_arr[13])
                # d = ord(res_arr[14])

                # a = res_arr[11]
                # b = res_arr[12]
                # c = res_arr[13]
                # d = res_arr[14]
                # # print('a')
                # add = (float(a + 256*b + 256**2*c + 256**3*d))
                # if self.enc_flag == 0:
                #     self.pre_add = add
                #     self.now_add = add
                #     self.enc_flag = 1
                # else:
                #     self.pre_add = self.now_add
                #     self.now_add = add

                # difference = self.now_add - self.pre_add
                # # print(difference)
                # if difference > 500:
                #     self.msg = self.pre_add

                # self.msg = add
                # self.pub_dis.publish(self.msg)
                # ############################################################################

                #차량의 속도 받아와 Ld설정
                self.control_data['look_ahead'] = 4
                self.speed_look_ahead = 5
                # self.control_data['look_ahead'] = (2/150) * self.control_data['target_speed'] + 3

                #pure pursuit
                self.control_data['steering'], self.curvature   = self.cal_steering(self.control_data['cur_x'], self.control_data['cur_y'],  self.control_data['cur_yaw'],  self.control_data['look_ahead'])
                # self.steering_err_old = self.steering_err
                # self.steering_err = self.control_data['steering'] - self.steering_veh
                # print('crvature : ', self.curvature)
                # 곡률로 V_ref 계산
                self.control_data['target_speed'] =  self.calc_velocity(self.curvature)
                # self.control_data['target_speed'] = 150
                self.V_err_old = self.V_err
                self.V_err = self.control_data['target_speed'] - self.V_veh
                # print('V_ref : ', self.control_data['target_speed'])
                # print('V_veh : ', self.V_veh)
                # print('V_err : ', self.V_err)
                
                # self.calc_lat_err()

                #V_ref, V_veh PID제어도 해볼예정(getOdoMsg 주기가 일정한지 확인 필요)
                self.t_old = self.t_new
                self.t_new = time.time()
                self.t_delta = self.t_new - self.t_old
                self.t = time.time() - self.t_start
                # print(self.t_delta)

                #
                #
                print(str(self.control_data['target_speed'])+','+str(self.V_veh)+','+str(int(self.V_in))+','+str(self.t))
                # print(self.control_data['steering'],",",self.steering_veh,",",self.t)
                # print(self.steering_veh)
                #
                #

                # self.steering_err_pro = self.Kp_s * self.steering_err
                # if self.Ki_v * self.V_err * self.t_delta < 100:
                #     self.steering_err_inte += self.Ki_s * self.steering_err * self.t_delta
                # self.steering_err_deri = self.Kd_s * (self.steering_err - self.steering_err_old) / self.t_delta
                # self.steering_in = self.steering_err_pro + self.steering_err_deri# + self.steering_err_inte 


                self.V_err_pro = self.Kp_v * self.V_err
                if self.Ki_v * self.V_err * self.t_delta < 100:
                    self.V_err_inte += self.Ki_v * self.V_err * self.t_delta
                self.V_err_deri = self.Kd_v * (self.V_err - self.V_err_old) / self.t_delta

                self.V_in = self.V_err_pro + self.V_err_deri + self.V_err_inte

                goal_dis = hypot(self.goal_x - self.control_data['cur_x'], self.goal_y - self.control_data['cur_y']) # 하이팟 대신에 x, y 따로 비교를 할까...
                if goal_dis < 5:
                    self.drive_mode = 'stop'
                else:
                    self.drive_mode = 'normal'

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

                # if self.V_in < self.control_data['target_speed'] and self.curvature <0.05:
                #     self.V_in = self.control_data['target_speed']
                
                # self.serWrite(int(self.V_in), self.V_veh, int(self.steering_in), cnt)
                self.serWrite(int(self.V_in), self.V_veh, int(self.control_data['steering']), cnt)
                # self.serWrite(int(self.control_data['target_speed']), self.V_veh, int(self.control_data['steering']), cnt)

    def run(self):
        # print("Controller ON")
        self.t_start = time.time()
        # rospy.Subscriber("/local", Local, self.getOdoMsg)
        rospy.Subscriber("/pose", Odometry, self.getOdoMsg)