#!/usr/bin/env python
# -*- coding:utf-8 -*-
from master_node.msg import Serial_Info  # 개발할 메세지 타입

from math import degrees, atan2, sin, radians, sqrt
import time


class General:
    def __init__(self, control):

        # 참조 수행
        self.cur = control.local  # local 좌표
        self.path = control.global_path  # global_path

        self.GeneralLookahead = control.lookahead  # 직진 주행시 lookahead

        self.serial_info = control.serial_info  #####  얘가 빈공간으로 들어오고 ㅣㅇ씅 @@@@@@@ 음 그냥  init 이라서 한번만 받아오는거네.같은 데이터 공간이어도 계속 받아와야 하징자ㅓㄹㄴㅁㅇ러ㅣㄷ렁마ㅣㄴ

        self.temp_msg = Serial_Info()

        self.past_mode = control.past_mode
        self.lookahead = 4
        self.speed_lookahead = 6
        self.WB = 1.04
        self.target_index = 0

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
        self.V_ref_max = 12

    def select_target(self, lookahead):
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

    # Dynamic Lookahead
    def Dynamic_LookAhead(self):
        self.lookahead = self.GeneralLookahead
        heading_difference = self.cur.heading - self.path.heading[self.target_index]
        if heading_difference > 10:
            self.lookahead = self.GeneralLookahead / 2
        # print ("LookAhead : ",self.lookahead)

    def pure_pursuit(self,point):
        # pure pursuit 계산되는 부분
        tmp_th = degrees(atan2((point.y - self.cur.y), (point.x - self.cur.x)))

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

    def PID(self, V_ref):

        self.V_err_old = self.V_err
        self.V_err = V_ref - self.serial_info.speed  ##외않대 ㅡ.ㅡ########

        # print('self.cur:',self.cur)
        # print('self.path',self.path)
        # print("self.serial_info.speed:", self.serial_info)

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

    def calc_k(self, k):
        critical_k = ((self.safety_factor / self.V_ref_max) ** 2) * 19.071

        if k < critical_k:
            V_ref = self.V_ref_max
        else:
            V_ref = self.safety_factor * (sqrt(19.071 / k))

        return V_ref  # km/h

    def calc_Vref(self):
        stidx = self.select_target(self.speed_lookahead)
        target_k = abs(self.path.k[stidx])
        # print(target_k)
        V_ref = self.calc_k(target_k)

        return int(V_ref)

    def calc_velocity(self):

        if self.past_mode != "general":  # 미션이 바뀔 때에는 변수리셋.
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
        if V_in > 20:
            V_in = 20
        elif V_in < V_ref:
            V_in = V_ref

        return int(V_in)

    def driving(self, control):
        # self.temp_msg=Serial_Info()
        # print('self.serial_info',self.serial_info.speed)
        if control.planning_info.mode == "general":
            self.V_ref_max = 12
        else:
            self.V_ref_max = 8
        self.temp_msg.steer = self.pure_pursuit(control.local_point)
        self.temp_msg.speed = self.calc_velocity()  # PID 추가 #   목표하는 스피드 넣어주는거  V_in 맞는데..
        self.temp_msg.brake = 0
        self.temp_msg.encoder = 0
        self.temp_msg.gear = 0
        self.temp_msg.emergency_stop = 0
        self.temp_msg.auto_manual = 1

        return self.temp_msg
