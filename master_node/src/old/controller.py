#-*- coding:utf-8 -*-

'''
self.control_data 는 실시간으로 master.py, sender.py와 공유됨 
'''

import pymap3d
#import csv
import rospy
from nav_msgs.msg import Odometry
from master_node.msg import Local
from std_msgs.msg import String
# from obstacle_detector.msg import Obs
import numpy as np
from math import radians
from math import degrees
from math import sin
from math import cos
from math import hypot
from math import atan2
# import LPP
import sys
import serial
import struct
import time
import matplotlib.pyplot as plt
from hybrid_a_star import path_plan 
from master_node.msg import Obstacles 
from master_node.msg import PangPang
from darknet_ros_msgs.msg import BoundingBoxes
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from std_msgs.msg import Float32

LocalPath = 0
GlobalPath = 1

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
        self.path_yaw = []
        self.lpath_x = []
        self.lpath_y = []
        #------------------------------------------------------
        self.gpaths=PointCloud()
        self.gpaths.header.frame_id='world'
        self.lpaths=PointCloud()
        self.lpaths.header.frame_id='world'
        self.cur_points=PointCloud()
        self.cur_points.header.frame_id = 'world'
        self.pub_gp=rospy.Publisher('/gpath',PointCloud,queue_size=1)
        self.pub_lp=rospy.Publisher('/lpath',PointCloud,queue_size=1)
        self.pub_c=rospy.Publisher('/cur_xy',PointCloud,queue_size=1)
        #------------------------------------------------------

        self.goal_x = 0
        self.goal_y = 0
        self.obx = []
        self.oby = []
        self.gtidx =0
        self.ltidx = 0
        self.gidx = 0
        self.ob_num = 0

        self.center = []
        self.goal_point = '9999'
        self.first = True # ground station이랑 최초인지 확인

        # self.next = False
        self.start_idx = 0
        self.ser = serial.Serial('/dev/ttyUSB0',115200) # USB 권한 주
        self.break_once = False
        self.person_detect = 0
        self.animation_cnt = 0
        self.old_lpath_x = []
        self.old_lpath_y = []
        self.mode_sw = GlobalPath
        self.LPP_is_done = 0
        self.GPP_is_done = 0
        self.old_obstacle = 0

        # UTurn
        self.mission_mode = ""
        self.first_uturn = True
        self.old_yaw = 0

        # Parking
        self.parking_first = True

        # Displacement - encoder
        self.msg = Float32()
        self.pub_dis = rospy.Publisher('/Displacement', Float32, queue_size=1)
        self.pre_add = 0 #엔코더에서 사용하는 저장변수
        self.now_add = 0 #엔코더에서 사용하는 저장변수
        self.enc_flag = 0 #엔코더에서 사용하는 flag


        # self.bring_tmp_map()
    
    def get_xy(self,  lat,  lon,  alt):
        e, n, u = pymap3d.geodetic2enu(lat, lon, alt, self.control_data['base_lat'],  self.control_data['base_lon'],  self.control_data['base_alt'])
        # print ('lat : ', lat, self.control_data['base_lat'])
        # print ('lon : ', lon, self.control_data['base_lon'])
        # print ('alt : ', alt, self.control_data['base_alt'])
        return e,  n

    def select_target(self, cx, cy, cidx, path_x, path_y):
        valid_idx_list = []

        for i in range(cidx, len(path_x)):
            dis = ((path_x[i]-cx)**2 +(path_y[i]-cy)**2)**0.5

            if dis <= self.control_data['look_ahead']:
                valid_idx_list.append(i)
            if len(valid_idx_list) != 0 and dis > self.control_data['look_ahead']:
                break
        if len(valid_idx_list) == 0:
            return 0
        else:
            return valid_idx_list[len(valid_idx_list) - 1]

             
    def cal_steering(self, cur_x, cur_y, cur_yaw, look_ahead):
        self.gtidx = self.select_target(cur_x, cur_y, self.gtidx, self.path_x, self.path_y)

        if self.mode_sw is GlobalPath:
            target_x = self.path_x[self.gtidx]
            target_y = self.path_y[self.gtidx]
        elif self.mode_sw is LocalPath:
            self.ltidx = self.select_target(cur_x, cur_y, self.ltidx, self.lpath_x, self.lpath_y)
            target_x = self.lpath_x[self.ltidx]
            target_y = self.lpath_y[self.ltidx]
            # target_x = self.lpath_x[10]
            # target_y = self.lpath_y[10]
        
        self.control_data['target_idx'] = self.gtidx

        return self.steering_angle(cur_x, cur_y, cur_yaw, target_x, target_y)
             
    def steering_angle(self,  cur_x, cur_y, cur_yaw, target_x, target_y):
        # pure pursuit 계산되는 부분 
        tmp_th = degrees(atan2((target_y - cur_y), (target_x - cur_x)))

        tmp_th = tmp_th%360

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
        goal_dis = hypot(self.goal_x - self.control_data['cur_x'], self.goal_y - self.control_data['cur_y']) # 하이팟 대신에 x, y 따로 비교를 할까...
        
        # if self.ls == 1 and self.break_once == False:
        #     # break_val = 0x10
        #     self.break_once = True
            # speed = 0x00

        if goal_dis > 3:
            speed = speed
        else:
            break_val = 0x0B #
            speed = 0x00
            # self.next = 
        get_test = []
        for i in range(int(self.ob_num)):
            get_test.append(self.calc_dis(self.obx[i], self.oby[i]))
        # ox, oy = [], []
        # if len(get_test) is not 0:
        #     if self.person_detect is 1 and min(get_test) < 8:
        #         speed = 0x00
        #         break_val = 150
        if self.person_detect == 1:
            print("emergency stop")
            break_val = 200
            # self.control_data['target_speed'] = 0

        if self.mode_sw == LocalPath:
            print("LPP on")
            # self.control_data['target_speed'] = 0
            # break_val = 0x0B
        if self.mission_mode == "PCOM":
            break_val = 200
            speed = 0x00

        if self.mission_mode == "UTURN":
            print("###Uturn ON")
            if self.first_uturn is False:
                print("okok", self.control_data['cur_yaw'], self.old_yaw)
                steering = -1999
                self.mission_mode = "UTing"

            else:
                pass


        # if self.
        print("steering value:", steering)
        result = struct.pack('!BBBBBBHhBBBB', 0x53, 0x54, 0x58, 0x01, 0x00, 0x00, int(speed),
                    steering, break_val, cnt, 0x0D, 0x0A )    # big endian 방식으로 타입에 맞춰서 pack   
        # print("pc : ", result[0], result[1], result[2], result[3], result[4], result[5], result[6], result[7], result[8],
        # result[9], result[10], result[11], result[12], result[13] )
        self.ser.write(result)

        if self.mission_mode == "UTing":
            print("###ing")
            time.sleep(3)
            self.mission_mode = ""
            self.first_uturn = False
        # print(result)


    def calc_dis(self, nx, ny):
        # print(nx, ny, )
        distance = ((nx - self.control_data['cur_x'])**2 +  (ny - self.control_data['cur_y'])**2)**0.5

        return distance

    def getObstacleMsg(self, msg):

        # print(self.control_data['target_idx'])
        if self.first == False:
            
            self.ob_num = msg.circle_number
            diff = self.ob_num - self.old_obstacle
            self.old_obstacle = self.ob_num

            self.obx, self.oby = [], []
            # print("gaesu" , len(msg.points))
            for pt in msg.points:
                self.obx.append(pt.x)
                self.oby.append(pt.y)

            if diff > 0:
                if self.LPP_is_done is 0:
                    # self.control_data['target_idx']
                    self.ltidx = 0
                    self.gidx, ox, oy = self.makeObstacleMap(self.control_data['cur_x'], self.control_data['cur_y'], self.control_data['cur_yaw'])
                    self.lpath_x, self.lpath_y = path_plan(self.control_data['cur_x'], self.control_data['cur_y'], self.control_data['cur_yaw'], self.path_x[self.gidx], self.path_y[self.gidx], self.path_yaw[self.gidx], ox, oy, self.animation_cnt)
                    self.mode_sw = LocalPath
                    if len(self.lpath_x) is not 0:
                        self.LPP_is_done = 1

                elif self.mode_sw is LocalPath:
                    get_test = []
                    for i in range(int(self.ob_num)):
                        get_test.append(self.calc_dis(self.obx[i], self.oby[i]))
                    ox, oy = [], []

                    if min(get_test) > 2:
                        self.gidx, ox, oy = self.makeObstacleMap(self.control_data['cur_x'], self.control_data['cur_y'], self.control_data['cur_yaw'])
                        self.lpath_x, self.lpath_y = path_plan(self.control_data['cur_x'], self.control_data['cur_y'], self.control_data['cur_yaw'], self.path_x[self.gidx], self.path_y[self.gidx], self.path_yaw[self.gidx], ox, oy, self.animation_cnt)
                #--------------------------------------------------------------------------------
                for i in range(len(self.lpath_x)):
                    lpath=Point32()
                    lpath.x=self.lpath_x[i]
                    lpath.y=self.lpath_y[i]
                    self.lpaths.points.append(lpath)
                self.lpaths.header.stamp=rospy.Time.now()
                self.pub_lp.publish(self.lpaths)
                print('lpaths published.')
                self.lpaths.points = []
                #--------------------------------------------------------------------------------

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

        return min_idx

    def ground(self, msg):
        print('from goal_node', msg.data)
        self.first = False
        self.goal_point = msg.data

    def GPP(self):
        # print('from goal_node:', msg.data)
        self.control_data['target_idx'] = 0
        self.path_x, self.path_y = [], []
        self.first = False

        self.start_idx = self.select_start_node(self.control_data['cur_x'], self.control_data['cur_y'], self.control_data['cur_yaw'])
        print(self.control_data['cur_x'], self.control_data['cur_y'], self.control_data['cur_yaw'])
        # if self.goal_point is not '9999':
        self.path_x, self.path_y, self.path_yaw = makeTarget.path_connect(str(self.start_idx), '34') # 출발노드, 도착노드

        self.goal_x, self.goal_y = self.path_x[-1], self.path_y[-1]

        #--------------------------------------------------------------------------------
        for i in range(len(self.path_x)):
            gpath=Point32()
            gpath.x=self.path_x[i]
            gpath.y=self.path_y[i]
            self.gpaths.points.append(gpath)
        self.gpaths.header.stamp=rospy.Time.now()
        self.pub_gp.publish(self.gpaths)
        print('gpaths published.')
        #-------------------------------------------------------------

    def select_goal_pt(self, ox, oy, c_yaw):

        tmpidx = self.gtidx 
        # print(tmpidx)

        while True:
            tmp_x = self.path_x[tmpidx]
            tmp_y = self.path_y[tmpidx]
            vec1 = (ox[1] - tmp_x, oy[1]- tmp_y)
            vec2 = (ox[0] - tmp_x, oy[0] - tmp_y)
            vec3 = (ox[2] - tmp_x, oy[2] - tmp_y)
            # vec4 = (ox[1] - tmp_x, oy[1] - tmp_y)
            # np.c
            tmp1 = np.cross(vec2, (cos(np.deg2rad(c_yaw)), sin(np.deg2rad(c_yaw))))
            tmp2 = np.cross(vec1, (cos(np.deg2rad(c_yaw)), sin(np.deg2rad(c_yaw))))
            tmp3 = np.cross(vec2, (cos(np.deg2rad(c_yaw - 90)), sin(np.deg2rad(c_yaw - 90))))
            tmp4 = np.cross(vec3, (cos(np.deg2rad(c_yaw - 90)), sin(np.deg2rad(c_yaw - 90))))

            # print("##", tmp1, tmp2, tmp3, tmp4)

            if tmp1 * tmp2 < 0 and tmp3 * tmp4 < 0:
                tmpidx += 1
            else:
                tmpidx -= 1
                return tmpidx-1



        # v_y = ox[30] - tmp_y


    def makeObstacleMap(self, c_x, c_y, c_yaw):
        ox, oy = [], [] # lidar, vision에서 받기

        # print('makeObstacleMap')
        # for i in range(100):
        #     ox.append(0.6)
        #     oy.append(i)

        # for i in range(100):
        #     ox.append(0.6)
        #     oy.append(i)

        ox.append(c_x-10)
        oy.append(c_y-1.0)
        ox.append(c_x+10)
        oy.append(c_y-1.0)

        ox.append(c_x-15)
        oy.append(c_y+26.0)
        ox.append(c_x+15)
        oy.append(c_y+26.0)
        

        for i in range(100):
            ox.append(c_x-10 + 0.2* i)
            oy.append(c_y - 1.0)


        for i in range(len(ox)):
            ox[i] -= c_x
            oy[i] -= c_y

        ox_rot, oy_rot = [], []
        rotate_yaw = c_yaw-90 % 360
        for i in range(len(ox)):
            ox_rot.append(ox[i]*cos(np.deg2rad(rotate_yaw)) + oy[i]*-sin(np.deg2rad(rotate_yaw)))
            oy_rot.append(ox[i]*sin(np.deg2rad(rotate_yaw)) + oy[i]*cos(np.deg2rad(rotate_yaw)))

        for i in range(len(ox)):
            ox_rot[i] += c_x
            oy_rot[i] += c_y

        for i in range(len(self.obx)):
            ox_rot.append(self.obx[i])
            oy_rot.append(self.oby[i])

        # print("#gidx: ", self.control_data['target_idx'] + 200, self.select_goal(c_x, c_y, c_yaw, ox, oy))
        # gidx = self.select_goal_pt(ox_rot, oy_rot, c_yaw)
        gidx = self.control_data['target_idx'] + 150
        # print("##############################################################", gidx)
        return gidx, ox_rot, oy_rot

    def calc_gp(self):
        return 12.718238, 12.762985, 4

    def parking(self):
        print("parking")
        gx, gy, gyaw = self.calc_gp()
        # print("???", self.control_data['cur_x']- gx, self.control_data['cur_y'] - gy)
        if self.parking_first == True:

            self.mode_sw = LocalPath
            self.control_data['target_speed'] = 30
            self.gidx, ox, oy = self.makeObstacleMap(self.control_data['cur_x'], self.control_data['cur_y'], self.control_data['cur_yaw'])
            self.lpath_x, self.lpath_y = path_plan(self.control_data['cur_x'], self.control_data['cur_y'], self.control_data['cur_yaw'], gx, gy, gyaw, ox, oy, self.animation_cnt)
            self.parking_first = False

        if abs(self.control_data['cur_x']- gx) <0.1 and abs(self.control_data['cur_y'] - gy) <0.1:
            self.mission_mode = "PCOM"

            # return 0

    def getOdoMsg(self,  msg):
        # cur_lon    = msg.pose.pose.position.x
        # cur_lat    = msg.pose.pose.position.y
        #print('cur_lon:', cur_lon, 'cur_lat:', cur_lat)
        # print("okokok")
        self.control_data['cur_yaw']  = msg.twist.twist.angular.z #imu 정북 기준으로 시계로 돌아갈때(시뮬)
        
        self.control_data['cur_x'], self.control_data['cur_y']  = msg.pose.pose.position.x, msg.pose.pose.position.y 

        # rviz #
        #--------------------------------------------------------
        cur_point = Point32()
        cur_point.x = self.control_data['cur_x']
        cur_point.y = self.control_data['cur_y']
        self.cur_points.points.append(cur_point)
        self.cur_points.header.stamp = rospy.Time.now()
        self.pub_c.publish(self.cur_points)
        #--------------------------------------------------------
        print('cur_x, cur_y', self.control_data['cur_x'], self.control_data['cur_y'], self.control_data['cur_yaw'])
        # print('cur_x, cur_y', self.control_data['cur_yaw'])

        # UTURN #
        #--------------------------------------------------------
        # if abs(self.control_data['cur_x'] - 17.3) < 0.8 and abs(self.control_data['cur_y'] - 36.4) < 0.8 and self.first_uturn == True and self.mission_mode is not "UTURN":
        #     print("UTURN ON")
        #     self.mission_mode = "UTURN"
        #     self.first_uturn = False
        #     self.old_yaw = self.control_data['cur_yaw']
        # #--------------------------------------------------------

        if self.first == False:
            if self.GPP_is_done is 0:
                print("GPP start")
                self.GPP()
                self.mode_sw = GlobalPath
                self.GPP_is_done = 1

            # 골포인트 성공할시
            if self.calc_dis(self.path_x[self.gidx], self.path_y[self.gidx]) < 4 and self.mode_sw is LocalPath:
                self.mode_sw = GlobalPath
                self.LPP_is_done = 0
                self.old_obstacle = 0
            # if self.calc_dis(-0.8059, 38.989) < 2
            #     self.mode_sw = GlobalPath
                # self.final()
                # self.ls = 1
            # print("first_check", self.control_data['first_check'])
            if (self.mode_sw is GlobalPath and self.GPP_is_done is 1) or (self.mode_sw is LocalPath and self.LPP_is_done is 1):
                # print("ls", self.ls, "gs", self.gs)
                # if len(self.lpath_x) is not 0:
                if self.mode_sw is GlobalPath:
                    # print("GlobalPath")
                    self.control_data['target_speed'] = 80
                elif self.mode_sw is LocalPath:
                    # print("LocalPath")
                    self.control_data['target_speed'] = 50                     

                
            print("before parking")
            if abs(self.control_data['cur_x'] - 12) < 7.5:
                self.parking()
                # print() 
            # print("after parking")
            self.control_data['steering']  = self.cal_steering(self.control_data['cur_x'], self.control_data['cur_y'],  self.control_data['cur_yaw'],  self.control_data['look_ahead'])

            # print("after parking2")
    # Serial
    #----------------------------------------------        
            cnt=0x00
            result = self.ser.readline()
            self.ser.flushInput()
            # print(result)
            # print(result[0])
            if (result[0] is 0x53 and result[1] is 0x54 and result[2] is 0x58):
                res_arr = []
                res_idx = 0
                # print('okokok')

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

                cnt = res_arr[15]
                # print("res_arr is", res_arr[6])
                self.serWrite(int(self.control_data['target_speed']), int(self.control_data['steering']), cnt)
               
                a = res_arr[11]
                b = res_arr[12]
                c = res_arr[13]
                d = res_arr[14]

                # print("I'm here" , a)
                add = (float(a + 256*b + 256**2*c + 256**3*d))
                # if self.enc_flag == 0:
                #     self.pre_add = add
                #     self.now_add = add
                #     self.enc_flag = 1
                # else:
                #     self.pre_add = self.now_add
                #     self.now_add = add

                # difference = self.now_add - self.pre_add
                
            

                self.msg = add

                # if difference > 500:
                #     self.msg = self.pre_add


                self.pub_dis.publish(self.msg)


    #----------------------------------------------



    def getObstacleClass(self,msg):
        self.person_detect = 0
        for os in msg.bounding_boxes:
            if(os.Class == "PERSON" and self.ob_num > 0):
                self.person_detect = 1
                # self.control_data['target_speed'] = 0x00
                print("person is detected")

    def uTurn(self, msg):
        if msg.data == "UTURN" and self.first_uturn is True:
            self.mission_mode = "UTURN"
            self.first_uturn = False
            self.old_yaw = self.control_data['cur_yaw']



    def run(self):
        print("Controller ON")
        #print(self.control_data)
        #print('controller')
        # rospy.Subscriber("/vehicle_node", String, self.final)
        # print("run ok")
        self.first = False
        # rospy.Subscriber('/goal_node', String, self.ground)
        # if self.first == False:
        # print('okokok')
        # rospy.Subscriber('/uturn', uuu, self.uTurn)
        rospy.Subscriber('/pangpang', PangPang, self.getObstacleMsg)
        rospy.Subscriber("/pose", Odometry, self.getOdoMsg)
        rospy.Subscriber('/darknet_ros/bounding_boxes_c',BoundingBoxes, self.getObstacleClass)


