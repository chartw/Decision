#!/usr/bin/env python
#-*- coding:utf-8 -*-
import threading

import rospy
# from sender import Sender
from controller_kcity import Controller

class Master:
    def __init__(self): # spin 에 의해 계속 이 class  실행./ 될때마다 init 반복. 
        rospy.init_node('manager', anonymous=False)
        
        #공유 데이터 => sender.py , controller.py 랑 실시간으로 값 공유 됨.
        self.control_data = { 'target_speed': 50,'steering':0,  #ccccccccccccccc
        'look_ahead': 4,'speed_look_ahead':5 ,'target_idx': 0, 'WB': 1, 
        # 'base_lat': 37.383784,  'base_lon': 126.654310,  'base_alt': 15.400, #송도운동장
        'base_lat': 37.239231667,  'base_lon': 126.773156667,  'base_alt': 15.400, #kcity initial
        # 'base_lat': 37.229,  'base_lon': 126.772971667,  'base_alt': 15.400, #1맵 initial
        'ob_cnt':0,
        'cur_x':0.0, 'cur_y':0.0, 'cur_yaw':0.0, 'first_check': True  }
        
        #self.sender = Sender(self)
        self.controller = Controller(self)

        #sender, controller 스레드 돌림.

        #th_sender = threading.Thread(target=self.sender.run, args=())
        th_controller = threading.Thread(target=self.controller.run, args=())
        
        #th_sender.start()
        th_controller.start()
        
        #th_sender.join()
        th_controller.join()
        
        # rospy.Rate(1000)
        rospy.spin() # master 가 계속 돌면 서 thread 로 실행 시키니까 controller 가 계속 run 되는거. 

if __name__ == "__main__":
	Master()
