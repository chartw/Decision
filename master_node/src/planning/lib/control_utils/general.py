#!/usr/bin/env python
# -*- coding:utf-8 -*-
from master_node.msg import Serial_Info  # 개발할 메세지 타입
from math import degrees, atan2, sin, radians, sqrt ,hypot
import time, rospy

#!/usr/bin/env python3
# -*- coding:utf-8 -*-
import rospy

import numpy as np
from math import radians, degrees, sin, cos, hypot, atan2, pi
import matplotlib.pyplot as plt

import sys
import time
from master_node.msg import Obstacles, PangPang, Planning_Info, Path, Local, Serial_Info
from nav_msgs.msg import Odometry
from lib.control_utils.pid import PID

# from darknet_ros_msgs.msg import BoundingBoxes
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from std_msgs.msg import Float32, Time, String, Int16


class General:
    def __init__(self, control):

        # 참조 수행
        self.cur = control.local  # local 좌표
        self.path = control.global_path  # global_path
        self.cur_idx = 0

        self.GeneralLookahead = control.lookahead  # 직진 주행시 lookahead

        self.serial_info = control.serial_info  #####  얘가 빈공간으로 들어오고 ㅣㅇ씅 @@@@@@@ 음 그냥  init 이라서 한번만 받아오는거네.같은 데이터 공간이어도 계속 받아와야 하징자ㅓㄹㄴㅁㅇ러ㅣㄷ렁마ㅣㄴ
        self.target_pub = rospy.Publisher("/target", PointCloud, queue_size=1)

        self.temp_msg = Serial_Info()
        self.past_mode = control.past_mode

        self.planning_info = control.planning_info
        self.mode = control.planning_info.mode

        self.WB = 1.04

        self.lookahead = 4
        # self.speed_lookahead = 0 # 안쓰임

        self.speed_idx = 0
        self.t_start = 0
        self.t_delta = 0
        self.t_old = 0
        self.t_new = 0
        self.t = 0

        self.Kp_v = 5
        self.Ki_v = 0.5
        self.Kd_v = 1

        self.V_err = 0
        self.V_err_old = 0
        self.V_err_pro = 0
        self.V_err_inte = 0
        self.V_err_deri = 0

        self.safety_factor = 0.8
        self.V_ref_max = 12  # 어차피 driving 에서 해서 의미없음.
        self.curve_flag = False

        self.first_check = True
        self.pid=PID()
        self.dist_list=[]
        self.dt_list=[0]
        self.target_list=[]
        self.vc_list=[]
        self.dt = 1.0 / 10.0


    def select_target(self, lookahead):  # 여기서 사용하는 self.path 관련정보를 바꾸면 됨. 여기서바꿔야하나?
        # min_dis = 99999
        # min_idx = 0
        # if self.first_check: # cur_idx 잡는데, 배달미션이나 cross 되는부분은
        #     for i in range(len(self.path.x)):
        #         dis = hypot(self.path.x[i]-self.cur.x,self.path.y[i]-self.cur.y)
        #         if min_dis > dis and abs(self.cur.heading-self.path.heading[i]) <30: # 여기에 등호가 붙으면, 뒷부분 index 잡고, 안붙으면 앞쪽 index
        #             min_dis = dis
        #             min_idx = i

        #     self.first_check = False
        # else:
        #     for i in range(max(self.cur_idx-50,0),self.cur_idx+50):
        #         dis = hypot(self.path.x[i]-self.cur.x,self.path.y[i]-self.cur.y)
        #         if min_dis > dis:
        #             min_dis = dis
        #             min_idx = i

        # self.cur_idx = min_idx # 차량과 가장 가까운 index.
        self.target_index = int(self.cur_idx + lookahead * 10)
        self.speed_idx = self.cur_idx + 60  # speed_idx는 무조건 이거로 가자.(speed_ld랑 상관없이)

    # def pure_pursuit(self,point):
    def pure_pursuit(self):

        # self.lookahead = 3
        if 10 < self.serial_info.speed < 20:
            self.lookahead = 0.2 * (self.serial_info.speed - 10) + 4  # 4로 바꾸기도 해.
        else:
            self.lookahead = 4
            # if self.path.k [self.speed_idx] >= 15 : # 속도 느린 직선구간.
            #     self.lookahead = 7
            # else:
            #     self.lookahead = 3 # 속도 느린 곡선구간 (좌회전, 우회전)

        self.select_target(self.lookahead)  # @@@@

        tmp_th = degrees(atan2((self.path.y[self.target_index] - self.cur.y), (self.path.x[self.target_index] - self.cur.x)))

        tmp_th = tmp_th % 360

        alpha = self.cur.heading - tmp_th
        if abs(alpha) > 180:
            if alpha < 0:
                alpha += 360
            else:
                alpha -= 360

        alpha = max(alpha, -90)
        alpha = min(alpha, 90)

        delta = degrees(atan2(2 * self.WB * sin(radians(alpha)) / self.lookahead, 1))

        if abs(delta) > 180:
            if delta < 0:
                delta += 360
            else:
                delta -= 360

        if abs(delta) >= 27.7:
            if delta > 0:
                return 27.7
            else:
                return -27.7
        else:

            return delta

    ###################조향 속도 구분선###################



    def calc_velocity(self, D_cur):

        D_ref = 8
        B_in = 0
        V_control = self.pid.run(D_ref,D_cur,self.serial_info.speed)
        V_control = min(V_control,8)
        self.dist_list.append(D_cur)
        self.target_list.append(D_ref)
        self.vc_list.append(V_control)
        plt.plot(self.dt_list,self.dist_list,'b')
        plt.plot(self.dt_list,self.target_list,'r')
        # plt.plot(self.dt_list,self.vc_list,'g')
        plt.pause(0.001)
        self.dt_list.append(self.dt_list[-1]+self.dt)
        print("vcontrol = ",V_control)
        V_in = self.serial_info.speed
        if V_control > 0:
            V_in+=V_control

        elif V_control < 0 :
            V_in+=V_control
            B_in = int(abs(V_control)*2.5)
            # B_in=0

        V_in = max(min(V_in, 8),0)
        B_in = min(B_in, 255)
        print("V_in",V_in,"B_in",B_in)
        print("D_cur = ",D_cur)
        return V_in, B_in

    def driving(self, control):
        self.mode = control.planning_info.mode
        self.cur_idx = control.planning_info.cur_index
        # self.temp_msg = Serial_Info()
        if self.mode == "general":
            self.temp_msg.speed, self.temp_msg.brake = self.calc_velocity(control.D_cur)      # 매개변수로 거리값
        elif self.mode=="general_left":
            self.temp_msg.speed = 12

        # elif self.mode == "kid":
        #     self.temp_msg.speed = self.calc_velocity()
        # elif self.mode == "small" or self.mode == "big":
        #     self.temp_msg.speed = self.calc_velocity()
        #     if control.serial_info.speed > self.temp_msg.speed + 1:
        #         self.temp_msg.brake=60
        # elif self.mode == "bump":
        #     self.temp_msg.speed = 8
        # elif self.mode == "pickup_complete" or self.mode == "drop_complete":
        #     self.temp_msg.speed = 16

            # if hypot(self.path.x[self.cur_idx]-self.cur.x,self.path.y[self.cur_idx]-self.cur.y)>1:
            #     self.temp_msg.speed = 8
            # else:
            #     self.temp_msg.speed = 13

        # self.temp_msg.steer = self.pure_pursuit(control.local_point)
        self.temp_msg.steer = self.pure_pursuit()
        # print("x,y",self.cur.x,self.cur.y)

        self.temp_msg.encoder = 0
        self.temp_msg.gear = 0
        self.temp_msg.emergency_stop = 0
        self.temp_msg.auto_manual = 1
        self.temp_msg.path_steer = self.path.heading[self.cur_idx]

        self.target = PointCloud()
        target_pt = Point32()
        target_pt.x = self.path.x[self.target_index]  # 이건지금
        target_pt.y = self.path.y[self.target_index]
        self.target.points.append(target_pt)
        self.target.header.frame_id = "world"
        self.target.header.stamp = rospy.Time.now()
        self.target_pub.publish(self.target)

        # print("ld",self.lookahead)
        print("ser_speed",self.serial_info.speed)

        print("cur_idx:", self.cur_idx, "ld:", round(self.lookahead, 2), "mode:", self.mode)
        # print("V_veh:", self.serial_info.speed)
        return self.temp_msg
