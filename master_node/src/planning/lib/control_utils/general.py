#!/usr/bin/env python
#-*- coding:utf-8 -*-
from math import degrees, atan2, sin, radians


class General:
    def __init__(self, control):
        # 깊은 복사 수행
        self.cur = control.local # local 좌표
        self.path = control.global_path # global_path
        self.GeneralLookahead = control.lookahead #직진 주행시 lookahead
        self.lookahead = None
        self.WB = 1
        self.target_index = 0
        

    def select_target(self):
        valid_idx_list = []

        for i in range(self.target_index, len(self.path.x)):
            dis = ((self.path.x[i] - self.cur.x) ** 2 + (self.path.y[i] - self.cur.y) ** 2) ** 0.5

            if dis <= self.lookahead:
                valid_idx_list.append(i)
            if len(valid_idx_list) != 0 and dis > self.lookahead:
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
        self.target_index = self.select_target()
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
