#!/usr/bin/env python
# -*- coding:utf-8 -*-
from master_node.msg import Obstacles, PangPang, Planning_Info, Path, Local, Serial_Info #@@@

from math import degrees, atan2, sin,cos, radians, sqrt ,hypot
import time ,rospy #@@@
from sensor_msgs.msg import PointCloud #@@@
from geometry_msgs.msg import Point32 #@@@
import numpy as np #@@@

class General:
    def __init__(self, control):

        # 참조 수행
        self.cur = control.local  # local 좌표
        self.path = control.global_path  # global_path # 한번만 받아오는거.  # self.path.heading 이 0~360 이어야 함. @@@@

        self.GeneralLookahead = control.lookahead  # 직진 주행시 lookahead

        self.serial_info = control.serial_info  # 10* km/h 상태.

        self.temp_msg = Serial_Info()

        self.past_mode = control.past_mode
        self.lookahead = 4
        self.speed_lookahead = 6
        self.WB = 1.04
        self.target_index = 0


        # For static_mission  # @@@@  control 에서 callback 받을 때마다 보내줘 여기로. # 계속 받아와야해서 init 말고 driving 에 있으면 될듯. ???#@@@@@@@@@@@
        self.obstacle_msg = Obstacles()
        # self.obstacle_msg.circles = [] # 메시지 지정만 하면 초기화 안해놔도 되겟지??? 
        # self.obstacle_msg.circle_number = 0






        self.head_x,self.head_y = 0,0
        # rviz for gpaths @@@@
        self.gpaths=PointCloud()
        self.gpaths.header.frame_id='world'
        self.pub_gp=rospy.Publisher('/gpath',PointCloud,queue_size=1)
        self.emergency = False




        ########## 종제어 
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
        #####################




    def select_target(self, lookahead): # 여기서 사용하는 self.path 관련정보를 바꾸면 됨. 여기서바꿔야하나? 
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

    # # Dynamic Lookahead
    # def Dynamic_LookAhead(self):
    #     self.lookahead = self.GeneralLookahead
    #     heading_difference = self.cur.heading - self.path.heading[self.target_index]
    #     if heading_difference > 10:
    #         self.lookahead = self.GeneralLookahead / 2
    #     # print ("LookAhead : ",self.lookahead)



    ############# lane_push 관련. ##########################################################################################


    def calc_nearest_obstacle(self):# 가장 작은 거리의 obstacle 정보 반환. @@@
        
        min_dis=1000000
        for idx,circle in enumerate(self.obstacle_msg.circle):
            dis = hypot(circle.center.x - self.head_x, circle.center.y - self.head_y) - circle.radius

            if dis < min_dis: # 더 작은게 있으면 그 point 의 distance, idx  저장. 
                min_dis = dis
                min_idx = idx

        return ( self.obstacle_msg.circle[min_idx].center.x ,
                 self.obstacle_msg.circle[min_idx].center.y ,
                 self.obstacle_msg.circle[min_idx].radius,
                 min_dis                                        )


    def push_direction(self,ox,oy): # ox,oy = 가장 가까운 장애물의 좌표.

        vector_obs  = np.array([ox,oy])  -  np.array([ self.path.x[self.target_index-40] , self.path.y[self.target_index-40] ])   # purple vector
        vector_path = np.array([ cos( radians(self.path.heading[self.target_index-40]) ), sin( radians(self.path.heading[self.target_index-40])) ])  # pink vector
        
        if np.cross(vector_obs,vector_path) >= 0: # lane 의 오른편에 장애물 위치.
            return True



    def lane_push(self): # push 된 lane 으로 개정. @@@@@@
        
        self.head_x = self.cur.x + 1.5*cos(radians(self.cur.heading))
        self.head_y = self.cur.y + 1.5*cos(radians(self.cur.heading))

        center_x, center_y, radius , emergency_d = self.calc_nearest_obstacle()  # 최소거리를 emergency_dis 로 받자.

        temp_rad = atan2( center_y - self.head_y , center_x - self.head_x) % 360 
        safe_d = emergency_d * sin(radians( abs(self.cur.heading - degrees(temp_rad)) )) - radius 
        d = 1.5 + 0.5/emergency_d  # 현속도 (self.serial_info.speed) , radius , emergency_d 에 맞게 수정 하기.
        L = 2                      # 일단 고정 / >> 속도 빠르면 멀면 길게잘라 

        ''' center_x,y, :  차 앞머리에서 가장 가까운 obstacle 의 정보.
            emergency_d :  차 앞부분과 최근인식된 장애물중점 사이 거리 - 장애물반지름 
            temp_rad    :  emergency_d line 의 각도 = 차량 앞부분에서 장애물까지 선이 이르는 각도  [0~ 2pi]
            safe_d      :  1m 이하일때만 실행! (멀리서 막 실행하지 않도록)_ 그 빗변삼각형의 높이 부분. 
            d[m]        :  push 길이 
            L[m]        :  쪼가리의 길이
        '''

        if emergency_d < 1:   # 1m 반경 들어오면, 무조건 비상 stop. -> 이후엔 수동으로 원상복귀 할거.
            self.emergency = True 

        else:                   # 여기에 추후에 차선정보도 포함시킬 수 있음
            self.emergency = False

            if emergency_d<4 and safe_d < 0.8: # 4m 이내로 진입했고, 진행방향과 충돌 위험이 있을 때에만, 경로 생성 함.(최종 조건)
                '''너무 자주 생성되는것을 대비하면, safe_d 를 조금 작게 ㄱㄱ'''

                for i in range( L*10 ): 
                    if self.push_direction(center_x,center_y): # 경로의 오른쪽에 있을때 왼쪽으로 push (차량 위치와 관계없이 경로기준 판단) 
                        self.path.x[ self.target_index + i ] -= d*cos(  radians(90) - radians( self.path.heading[self.target_index + i]) )
                        self.path.y[ self.target_index + i ] += d*sin(  radians(90) - radians( self.path.heading[self.target_index + i]) )
                    else:
                        self.path.x[ self.target_index + i ] += d*cos(  radians(90) - radians( self.path.heading[self.target_index + i]) )
                        self.path.y[ self.target_index + i ] -= d*sin(  radians(90) - radians( self.path.heading[self.target_index + i]) )
         
                ########## 딱 lane_push 될 때에만 rviz로 송출 ------------------------------
                for i in range( self.target_index- 300,self.target_index + 300)): # 앞뒤 30m 씩 까지만 path 가시화! _ path가 계속 바뀌어야함!
                    gpath = Point32()
                    gpath.x=self.path.x[i]
                    gpath.y=self.path.y[i]
                    self.gpaths.points.append(gpath)
                self.gpaths.header.stamp=rospy.Time.now()
                self.pub_gp.publish(self.gpaths)

                print('gpaths published._ in general.py')
                #-------------------------------------------------------------



    def pure_pursuit(self):

        # if self.path_mission == 'static': # kcity csv의 마지막 부분 작업 이후에.추가.

        if self.obstacle_msg.circles: ## control에서 오는거에 장애물이 담겨있으면, @@@@@@@
            self.lane_push() # class 전역변수만 바꿔줌
			
        
        self.target_index = self.select_target(self.lookahead)

        target_x = self.path.x[self.target_index]
        target_y = self.path.y[self.target_index]

        # self.Dynamic_LookAhead() # 동적 lookAhead

        if len(self.path.x) == 0: # 경로 없으면 직진해라? 굳이 왜 썼을까.. 
            return 0

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







    ###################속도 종제어.################################3

    def PID(self, V_ref):

        self.V_err_old = self.V_err
        self.V_err = V_ref - self.serial_info.speed  ## pid  이상할땐, 여기 10 곱해진 상태.

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
        V_ref = self.calc_k(target_k) # km/h

        return 10 * int(V_ref)

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

        V_ref = self.calc_Vref() # 10*km/h
        V_in = self.PID(V_ref) # 10*km/h
        if V_in > 200:
            V_in = 200
        elif V_in < V_ref:
            V_in = V_ref

        return int(V_in) # 10* km/h


##################################################3







    def driving(self, control):
        # @@@@ control에서 callback받아서 , 계속 여기로 전해줄거.
        self.obstacle_msg.circles = control.obstacle_msg.circles
        self.obstacle_msg.circle_number = control.obstacle_msg.circle_number 


        # 미션별 최고속도. 여기에 ??
        if control.planning_info.mode == "general":
            self.V_ref_max = 12
        else:
            self.V_ref_max = 8
        ########################### 

        
        self.temp_msg.steer = self.pure_pursuit()
        self.temp_msg.speed = self.calc_velocity()  # PID 추가 # 
        self.temp_msg.brake = 0
        self.temp_msg.encoder = 0
        self.temp_msg.gear = 0
        self.temp_msg.emergency_stop = 0
        self.temp_msg.auto_manual = 1
        
        # prerequisite 
        if self.emergency:
            self.temp_msg.emergency_stop = 1

        return self.temp_msg