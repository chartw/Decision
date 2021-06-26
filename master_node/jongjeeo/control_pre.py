#-*- coding:utf-8 -*-

'''
self.control_data 는 실시간으로 master.py, sender.py와 공유됨 
'''

import pymap3d
#import csv
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import String
# from obstacle_detector.msg import Obs
import numpy as np
from math import radians
from math import degrees
from math import sin
from math import hypot
from math import atan2
# import LPP
import sys
import serial
import struct
import matplotlib.pyplot as plt
from hybrid_a_star import path_plan 

# sys.path.append("./map/")

# try:
#     import GPP
# except:
#     raise

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
        # self.next = False
        self.start_idx = 0
        self.ser = serial.Serial('/dev/ttyUSB0',115200) # USB 권한 주
        self.ls = 0

        # self.bring_tmp_map()
        
    def get_xy(self,  lat,  lon,  alt):
        e, n, u = pymap3d.geodetic2enu(lat, lon, alt, self.control_data['base_lat'],  self.control_data['base_lon'],  self.control_data['base_alt'])
        # print ('lat : ', lat, self.control_data['base_lat'])
        # print ('lon : ', lon, self.control_data['base_lon'])
        # print ('alt : ', alt, self.control_data['base_alt'])
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

             
    def cal_steering(self, cur_x, cur_y, cur_yaw, look_ahead):

        tidx = self.select_target(cur_x, cur_y, self.control_data['target_idx'])
        
        target_x = self.path_x[tidx]
        target_y = self.path_y[tidx]
        # print("#target x, y: ", target_x, target_y)
        
        # print('target_x: ',  target_x,  'target_y:',  target_y)
        # print('tidx:', tidx)
        
        # dis = hypot(target_x - cur_x,  target_y - cur_y)

        return self.steering_angle(cur_x, cur_y, cur_yaw, target_x, target_y)
    # def first_steering
        # if dis <= self.control_data['look_ahead']:
        #     # print('########################short:',  self.control_data['target_idx'])
        #     # if(self.control_data['target_idx'] < len(self.path_x))-50:
        #     self.control_data['target_idx'] += 1
        #     return self.cal_steering(cur_x, cur_y, cur_yaw, look_ahead)
        # else : 
        #     # print('hit the target:',  dis)
        #     return self.steering_angle(cur_x, cur_y, cur_yaw, target_x, target_y)
             
    def steering_angle(self,  cur_x, cur_y, cur_yaw, target_x, target_y):
        # pure pursuit 계산되는 부분 
        tmp_th = degrees(atan2((target_y - cur_y), (target_x - cur_x)))
        # print('cur_x:',  cur_x ,  'cur_y',  cur_y,  'target_x:',  target_x,  'target_y:',  target_y,  'atan2((target_y - cur_y), (target_x - cur_x):',  tmp_th)
        # print('-'*30)
        #print('original target theta:',  tmp_th)
        #print('cur_yaw:',  cur_yaw)
        # print(target_x,target_y)

        tmp_th = tmp_th%360
        #print('360 target theta:',  tmp_th)
        
        
        # if (tmp_th > 90) and (tmp_th <= 360):
        #     tmp_th -= 90
        # else:
        #     tmp_th += 270
        #print('changed 360 target theta:',  tmp_th)
        # print('cur_yaw:',  cur_yaw,  'target_angle:',  tmp_th)
        alpha =  cur_yaw - tmp_th  
        # print('alpha:',  alpha)
        # print('tmp_th', tmp_th)
        if abs(alpha)>180:
            if (alpha < 0) :
                alpha += 360
            else :
                alpha -= 360
        # print('alpha2:',  alpha)        
        alpha = max(alpha,  -90)
        alpha = min(alpha,  90)
        # print('modified alpha:',  alpha)
        # alpha = -alpha
        delta = degrees(atan2(2*self.control_data['WB']*sin(radians(alpha))/self.control_data['look_ahead'],1))
        # print('delta:',  delta)
        
        #test = tmp_th - cur_yaw
        
        
        if abs(delta)>180:
            if (delta < 0) :
                delta += 360
            else :
                delta -= 360
        
        
        # print("delta2 : ", delta)
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
        # print('speed : ', speed);
        # print("OUT_PUT | speed : ", speed, ', steering : ',steering, ', break : ', break_val)\
        # print(self.goal_x)
        # print("#x,y,steer is: ", self.control_data['cur_x'], self.control_data['cur_y'], steering)
        goal_dis = hypot(self.goal_x - self.control_data['cur_x'], self.goal_y - self.control_data['cur_y']) # 하이팟 대신에 x, y 따로 비교를 할까...
        
        # if self.ls == 3:
        #     break_val = 0x20
        #     speed = 0x00

        if goal_dis > 3:
            speed = speed
        else:
            break_val = 0x0B #
            speed = 0x00
            # self.next = True

        result = struct.pack('!BBBBBBHhBBBB', 0x53, 0x54, 0x58, 0x01, 0x00, 0x00, int(speed),
                    steering, break_val, cnt, 0x0D, 0x0A )    # big endian 방식으로 타입에 맞춰서 pack   
        # print("pc : ", result[0], result[1], result[2], result[3], result[4], result[5], result[6], result[7], result[8],
        # result[9], result[10], result[11], result[12], result[13] )
        self.ser.write(result)

    # def serWrite(self, speed, steering, cnt):
    #     input_speed = speed
    #     break_val = 0x01
    #     # steering 값 2000 넘길 시 2000으로 설정
    #     if abs(steering)>=2000:
    #         if steering>0:
    #             steering = 1999
    #         else :
    #             steering =-1999
    #     # if abs(steering) < 300 :
    #     #     steering = int(0.5*steering)
       
    #     # 기어 기본값 0: 전진, 1:후진
    #     # print('speed : ', speed);
    #     # print("OUT_PUT | speed : ", speed, ', steering : ',steering, ', break : ', break_val)\
    #     # print(self.goal_x)
    #     # print("#x,y is: ", self.control_data['cur_x'], self.control_data['cur_y'])
    #     goal_dis = hypot(self.goal_x - self.control_data['cur_x'], self.goal_y - self.control_data['cur_y']) # 하이팟 대신에 x, y 따로 비교를 할까...
    #     if(goal_dis > 3):
    #         speed = speed
    #     else:
    #         break_val = 0x11
    #         speed = 0x00
    #     # else:
    #     # print("### brake")
    #     # if abs(self.control_data['steering']) < 200 and goal_dis < 5:
    #     #     print("### steer brake")
    #     #     break_val = 0x30
    #     #     speed = 0x00

    #     # elif goal_dis < 3: 
    #     #     print("###else brake")
    #     #     break_val = 0x0F #
    #     #     speed = 0x00
    #     # self.next = True

    #     result = struct.pack('!BBBBBBHhBBBB', 0x53, 0x54, 0x58, 0x01, 0x00, 0x00, int(speed),
    #                 steering, break_val, cnt, 0x0D, 0x0A )    # big endian 방식으로 타입에 맞춰서 pack   
    #     # print("pc : ", result[0], result[1], result[2], result[3], result[4], result[5], result[6], result[7], result[8],
    #     # result[9], result[10], result[11], result[12], result[13] )
    #     self.ser.write(result)

    # def getObstacleMsg(self, msg):
    #     #라이다에서 장애물 받아오는 콜백 함수인데.. LPP는 연구중... ㅠ 
    #     #추후에 테스트 한다면 지금 한번에 받아오는 200개 가량의 장애물이 잘 처리가 되는지가 관건일듯.. 
    #     ob = []s
    #     for pt in msg.points:
    #         ob.append([pt.x, pt.y])

    #     ob = np.array(ob)

    #     print(ob)
    #     print(len(ob))
 
    # def final(self, msg):
    #     print(msg.data)
    #     print(type(msg.data))
    #     print(msg.data[0]+msg.data[1])
    #     print(msg.data[2]+msg.data[3])
    #     #self.path_x, self.path_y = makeTarget.path_connect(msg.data[0]+msg.data[1], msg.data[2]+msg.data[3]) # 출발노드, 도착노드 
    #     self.path_x, self.path_y = makeTarget.path_connect("50", "32") # 출발노드, 도착노드     
    #     self.goal_x, self.goal_y = self.path_x[-1], self.path_y[-1]
    #     print("self.goal_x: ", self.goal_x)
    #     # print('path_x:',  self.path_x)
    #     # self.control_data['gpp_check'] = False
    #     self.control_data['first_check'] = False
    #     self.closest_pt(self.control_data['cur_x'], self.control_data['cur_y'], self.control_data['cur_yaw'])

    def calc_dis(self, nx, ny):
        
        distance = ((nx - self.control_data['cur_x'])**2 +  (ny - self.control_data['cur_y'])**2)**0.5

        return distance


    def select_start_node(self, cx, cy, cyaw):
        nodelist = []
        nodelist = makeTarget.node_set()
        # print(nodelist)
        min_dis = 99999
        min_idx = 10000
        temp_idx = 10000
        temp_dis = 9999

        # valid_node_list = []
        for node in nodelist:
            # dx, dy = nodelist[node].x - cx, nodelist[node].y - cy
            # dyaw = (atan2(dy, dx) *180 / 3.14) - 90
            # if dyaw < 0:
            #     dyaw += 360
            # print(node, dyaw, cyaw)

            # if abs(dyaw - cyaw) < 10:
            #     valid_node_list.append(nodelist[node])
            temp_dis = self.calc_dis(nodelist[node].x, nodelist[node].y)
            if temp_dis < min_dis:
                min_dis = temp_dis
                min_idx = node




        # # print(valid_node_list)

        # for node in valid_node_list:
        #     # print("########", nodelist[node].x, nodelist[node].y)
        #     # print(node)
        #     temp_dis = self.calc_dis(valid_node_list[node].x, valid_node_list[node].y)
        #     print(temp_dis)
        #     if temp_dis < min_dis:
        #         min_dis = temp_dis
        #         min_idx = node
        #     # print(min_i)

        return min_idx

    def final(self):
        self.control_data['target_idx'] = 0
        self.path_x, self.path_y = [], []
        # self.path_x, self.path_y = makeTarget.path_connect(msg.data[0]+msg.data[1], msg.data[2]+msg.data[3]) # 출발노드, 도착노드 
        
        # self.select_start_node()
        self.start_idx = self.select_start_node(self.control_data['cur_x'], self.control_data['cur_y'], self.control_data['cur_yaw'])
        # print("### start_idx: ", self.start_idx)
        # print(msg.data)
        # self.path_x, self.path_y = makeTarget.path_connect(str(self.start_idx),msg.data) # 출발노드, 도착노드     
        self.path_x, self.path_y, self.path_k = makeTarget.path_connect(str(self.start_idx), "47") # 출발노드, 도착노드    
        # self.path_x, self.path_y = makeTarget.path_connect("50", "32") # 출발노드, 도착노드     
        self.goal_x, self.goal_y = self.path_x[-1], self.path_y[-1]
        # print("self.goal_x: ", self.goal_x)
        # print('path_x:',  self.path_x)
        # self.control_data['gpp_check'] = False
        self.control_data['first_check'] = False
        # self.closest_pt(self.control_data['cur_x'], self.control_data['cur_y'], self.control_data['cur_yaw'])


        
    def getOdoMsg(self,  msg):
        cur_lon    = msg.pose.pose.position.x
        cur_lat    = msg.pose.pose.position.y
        #print('cur_lon:', cur_lon, 'cur_lat:', cur_lat)

        # print("okokok")
        self.control_data['cur_yaw']  = msg.twist.twist.angular.z #imu 정북 기준으로 시계로 돌아갈때(시뮬)
        # self.control_data['cur_yaw'] = 360 - msg.twist.twist.angular.z #imu 정북 기준으로 시계로 돌아갈때(송도)
        # print(msg.pose.pose.position.z)
        
        #print('lon:',  cur_lon,  'lat:',  cur_lat, 'yaw:',  self.control_data['cur_yaw'])
        
        g_x, g_y, g_yaw = 22.9, 43.8, 329 # 의미없음 나중에 지우자...
        self.control_data['cur_x'], self.control_data['cur_y']  = self.get_xy(cur_lat,  cur_lon,  self.control_data['base_alt'])
        print(self.control_data['cur_x'],",",self.control_data['cur_y'])
        # if abs(self.control_data['cur_x']-9.5) < 10 and abs(self.control_data['cur_y']- 21.7) < 10 and self.ls is 0:
        #     print("LPP mode start###")
        #     self.path_x, self.path_y = path_plan(self.control_data['cur_x'], self.control_data['cur_y'], self.control_data['cur_yaw'], g_x, g_y, g_yaw)
        #     # print("####print",self.path_x, "\n####yyyy", self.path_y)
        #     self.ls = 1

        # elif abs(self.control_data['cur_x']-23) < 10 and abs(self.control_data['cur_y']- 43.3) < 10 and self.ls is 1:
        #     print("LPP mode over###")
        #     self.final()
        #     self.ls = 3
            # self.ls = 3

        

        if self.control_data['first_check'] is False:
            #print('get msg')

            #print('x:', self.control_data['cur_x'], 'y:', self.control_data['cur_y'])
            # print( 'x:',  self.control_data['cur_x'], 'y:',  self.control_data['cur_y'],  'yaw:',  self.control_data['cur_yaw'])
            # print('index:',  self.control_data['target_idx'])
            # print('cur yaw :',  self.control_data['cur_yaw'])
            self.control_data['steering']  = self.cal_steering(self.control_data['cur_x'], self.control_data['cur_y'],  self.control_data['cur_yaw'],  self.control_data['look_ahead'])
            # if self.control_data['steering'] is 0x0A:
            #     self.control_data['steering'] = 0x0B
            # print('steering : ', self.control_data['steering'])
            
            # print(result)
            cnt=0x00
            
            # resultarr = []
            result = self.ser.readline()
            self.ser.flushInput()
            # print("result[0] is]", result[0])
            if (result[0] is 0x53 and result[1] is 0x54 and result[2] is 0x58):
                res_arr = []
                res_idx = 0

                while True:
                    # print("result is", result)
                    for i in range(len(result)):
                        # print(result[i])
                        # print(res_idx)f
                        if result[i] is 0x0A and i is not 17:
                            # print("### 0x0A Found!", i, "th data")
                            res_arr.append(0x0B)
                        else :
                            # print("### append", res_arr)
                            res_arr.append(result[i])
                        # print("res_idx is", res_idx)
                        # res_idx += 1

                    if len(res_arr) < 18:
                        # print("### add line")
                        result = self.ser.readline()
                    else:
                        break

                cnt = res_arr[15]
                # print("res_arr is", res_arr)
                self.serWrite(int(self.control_data['target_speed']), int(self.control_data['steering']), cnt)

            # if (result[0] is 0x53 and result[1] is 0x54 and result[2] is 0x58):
            #     if len(result) > 17:
            #         # print("over 17")
            #             # print("erp : ", result[0], result[1], result[2], result[3], result[4], result[5], result[6], result[7], result[8],
            #             # result[9], result[10], result[11], result[12], result[13], result[14], result[15], result[16], result[17])
            #         cnt = result[15]
            #         # print(cnt)
            #         # look_ahead = cal_lookahead(result[6])
            #         self.serWrite(int(self.control_data['target_speed']), int(self.control_data['steering']), cnt)
            #         # cnt가 10일때 패킷이 잘려서 2번에 걸쳐 들어옴.+++++++++++++++++++++++++++++ 0x0a값이 아스키코드 LF(new line)!!!
            #     elif len(result) == 16:
            #         # print("16")
            #         # print("erp : ", result[0], result[1], result[2], result[3], result[4], result[5], result[6], result[7], result[8],
            #         # result[9], result[10], result[11], result[12], result[13], result[14], result[15])
            #         # add_result = self.ser.readline() # erp -> pc
            #         # print(cnt)
            #         self.serWrite(int(self.control_data['target_speed']), int(self.control_data['steering']), 10)
            #     # else :
                #     print('error')
                #     print(result)

                # while True:

                #     if len(resultarr)!=18:
                #         for i in range(len(result)):
                #             resultarr.append(result[i])
                #     else:
                #         break

                #     result = self.ser.readline() # erp -> pc                    
                #     self.ser.flushInput()

                # self.serWrite(int(self.control_data['target_speed']), int(self.control_data['steering']), resultarr[15])

        else:
            pass
        #self.control_data['steering'] = 1900         

        if self.final_switch == 0:
            self.final()
            self.final_switch = 1


    def run(self):
        # print("Controller ON")
        #print(self.control_data)
        #print('controller')
        # rospy.Subscriber("/vehicle_node", String, self.final)
        # print("run ok")
        rospy.Subscriber("/sim_gps", Odometry, self.getOdoMsg)
        # rospy.Subscriber('/Obs', Obs, self.getObstacleMsg)



