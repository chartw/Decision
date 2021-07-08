#!/usr/bin/env python
#-*- coding:utf-8 -*-
from master_node.msg import Serial_Info  # 개발할 메세지 타입

from math import degrees, atan2, sin, radians
import time


class General:
    def __init__(self, control):
        # 깊은 복사 수행
        self.cur = control.local # local 좌표
        self.path = control.global_path # global_path
        self.GeneralLookahead = control.lookahead #직진 주행시 lookahead
        self.serial_info = control.serial_info
        self.past_mode = control.past_mode

        self.lookahead = None
        self.speed_lookahead = 6
        self.WB = 1.04
        self.target_index = 0

        self.t_start = 0
        self.t_delta = 0
        self.t_old = 0
        self.t_new = 0
        self.t = 0

        self.Kp_v = 50
        self.Ki_v = 5
        self.Kd_v = 10

        self.V_err = 0
        self.V_err_old = 0
        self.V_err_pro = 0
        self.V_err_inte = 0
        self.V_err_deri = 0

        self.safety_factor = 0.8
        self.V_ref_max = 150        

    def select_target(self,lookahead):
        valid_idx_list = []

        for i in range(self.target_index, len(self.path.x)):
            dis = ((self.path.x[i] - self.cur.x) ** 2 + (self.path.y[i] - self.cur.y) ** 2) ** 0.5

            if dis <= self.lookahead:
                valid_idx_list.append(i)
            if len(valid_idx_list) != 0 and dis > lookahead:
                break
        if len(valid_idx_list) == 0:
            return 0
        else:
            return valid_idx_list[len(valid_idx_list) - 1]

    #Dynamic Lookahead
    def Dynamic_LookAhead(self):
        self.lookahead = self.GeneralLookahead
        heading_difference = (self.cur.heading - self.path.heading[self.target_index])
        if heading_difference > 10 :
            self.lookahead = self.GeneralLookahead/2 
        print ("LookAhead : ",self.lookahead)

    def pure_pursuit(self):
        
        self.Dynamic_LookAhead() # 동적 lookAhead 

        if len(self.path.x)==0: 
            return
        self.target_index = self.select_target(self.lookahead)
        # print(self.cur.x, self.cur.y)

        target_x = self.path.x[self.target_index]
        target_y = self.path.y[self.target_index]
        # pure pursuit 계산되는 부분
        tmp_th = degrees(atan2((target_y - self.cur.y), (target_x - self.cur.x)))

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

    def PID(self,V_ref):
        
        self.V_err_old = self.V_err
        self.V_err = V_ref - self.serial_info.speed

        self.t_old = self.t_new
        self.t_new = time.time()
        self.t_delta = self.t_new - self.t_old
        self.t = time.time() - self.t_start

        self.V_err_pro = self.Kp_v * self.V_err
        if self.Ki_v * self.V_err * self.t_delta < 100:
            self.V_err_inte += self.Ki_v * self.V_err * self.t_delta
        self.V_err_deri = self.Kd_v * (self.V_err - self.V_err_old) / self.t_delta

        V_in = self.V_err_pro + self.V_err_inte + self.V_err_deri

        return V_in


    def calc_velocity(self, k):
        critical_k = ((self.safety_factor/self.V_ref_max)**2) * 19.071

        if k < critical_k:
            V_ref = self.V_ref_max
        else:
            V_ref = self.safety_factor * (sqrt(19.071/k))

        return 10 * V_ref # 10*(km/h)


    def calc_Vref(self):
        stidx = self.select_target(self.speed_lookahead)
        target_k = abs(self.path.k[stidx])
        V_ref = self.calc_velocity(target_k):

        return int(V_ref)


    def calc_velocity(self):
       
        if self.past_mode != 'general':
            # 다른 미션에서 general로 왔을때 pid 변수초기화
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

        V_ref = self.calc_Vref()
        V_in = self.PID(V_ref)
        if V_in > 200:
            V_in = 200
        elif V_in < V_ref:
            V_in = V_ref

        return int(V_in)

    def driving(self):
        temp_msg=Serial_Info()
        temp_msg.steer = self.pure_pursuit()
        temp_msg.speed = self.calc_velocity()  # PID 추가
        temp_msg.brake = 0
        temp_msg.encoder = 0
        temp_msg.gear = 0
        temp_msg.emergency_stop = 0
        temp_msg.auto_manual = 1

        return temp_msg