#!/usr/bin/env python
# -*-coding: utf-8-*-

import serial
import time 
import struct
from math import radians
from math import degrees
from math import cos
from math import sin
from math import asin
from math import pow
from math import atan2
from morai_udp_parser import erp_udp_parser,erp_udp_sender
import csv
from time import sleep
import rospy
from master.msg import sync_msg
import socket #UDP LAN
from cam_util import get_img
import cv2


ser = serial.Serial('/dev/ttyUSB0', 115200)
# ser = serial.Serial('/dev/pts/16', 38400)
ctrl_cmd=erp_udp_sender('127.0.0.1',7601)

msg_sync = sync_msg()
pub = rospy.Publisher('/erp_status', sync_msg, queue_size = 1)
# path_x = [ 0, 0, 0, 10];
# path_y = [ 10, 20, 30, 40 ];  
path_x = [ ];
path_y = [ ];
path_k = [ ];
path_type = [ ];
path_idx = 0;
look_ahead = 40
speed_lim = 7
first_chk = True
k = 0.3
cur_speed = 0



#gear 0:D, 1:N, 2:R
def send_packet(steering, speed, break_val, gear):
    result = ser.readline() # erp -> pc
    steering = (steering*71)
    if abs(steering)>2000:
        if steering>0:
            steering = 2000
        else :
            steering =-2000
    speed *= 10
    if len(result) >= 15: 
        cnt = result[15]
        result = struct.pack('!BBBBBBHhBBBB', 0x53, 0x54, 0x58, 0x01, 0x00, gear, int(speed),
                    int(steering), int(break_val), cnt, 0x0D, 0x0A )    # big endian 방식으로 타입에 맞춰서 pack   
        # print(cnt, result, steering, speed)
        ser.write(result)
    sleep(0.1)



def action_lane_right():
    global path_idx
    start_t = time.time()# 시작 시간 저장
    while((time.time()-start_t)<2):
        send_packet(0,0, 50, 0)  
    start_t = time.time()# 시작 시간 저장
    while((time.time()-start_t)<5.5):
        send_packet(30, 7, 0, 0)  
    start_t = time.time()
    while((time.time()-start_t)<1.5):
        send_packet(0, 2, 0, 0)
    start_t = time.time()
    while((time.time()-start_t)<5.5):
        send_packet(-30,7, 0, 0)
    start_t = time.time()
    while((time.time()-start_t)<1):
        send_packet(0, 5, 0, 0)
    path_idx += 10


def action_lane_left():
    global path_idx
    start_t = time.time()# 시작 시간 저장
    while((time.time()-start_t)<1):
        send_packet(0,0, 50, 0)  
    start_t = time.time()# 시작 시간 저장
    while((time.time()-start_t)<3):
        send_packet(-30, 11, 0, 0)  
    start_t = time.time()
    while((time.time()-start_t)<1):
        send_packet(0, 5, 0, 0)
    start_t = time.time()
    while((time.time()-start_t)<1.5):
        send_packet(30, 11, 0, 0)
    start_t = time.time()
    while((time.time()-start_t)<1):
        send_packet(0, 5, 0, 0)
    path_idx += 10


def action_parking():
    global path_idx
    start_t = time.time()# 시작 시간 저장
    while((time.time()-start_t)<2):
        send_packet(0,0, 50, 0)  

    start_t = time.time()
    while((time.time()-start_t)<5.5):
        send_packet(30,7, 0, 0)

    start_t = time.time()
    while((time.time()-start_t)<1.5):
       send_packet(0,2, 0, 0)

    start_t = time.time()
    while((time.time()-start_t)<5):
       send_packet(0,0, 60, 0)

    start_t = time.time()
    while((time.time()-start_t)<7):
       send_packet(0,6, 0, 2)

    start_t = time.time()
    while((time.time()-start_t)<3.5):
        send_packet(30, 7, 0, 2)  

    start_t = time.time()# 시작 시간 저장
    while((time.time()-start_t)<2.5):
        send_packet(0, 0, 50, 0)  
    path_idx += 10


def action_break_crosswalk(cnt):
    start_t = time.time()
    while((time.time()-start_t)<5):
        send_packet(0,0, 0, 0)


def serWrite(speed, steering, cnt, cur_mode, cur_speed):
    input_speed = speed
    break_val = 0x00
    if cur_mode == 'ctrl_normal':
        break_val = 0x01
    elif cur_mode=='ctrl_slow_1':
        speed = 50
        break_val = 0x01
    elif cur_mode=='ctrl_slow_2':
        speed = 40
        break_val = 0x01
    elif cur_mode=='ctrl_slow_3':
        speed = 30
        break_val=0x20
    elif cur_mode== 'ctrl_break':
        print('start_break')
        speed = 0
        break_val=20
        print('break | cur_speed:',cur_speed)
        if cur_speed <80:
            print('===========break_A')
            break_val = input_speed - int(0.4 *cur_speed)
            if break_val <= 0:
                break_val = 1
        elif cur_speed <100:
            print('===========break_B')
            break_val = input_speed + 10 - int(0.3 *cur_speed)
            if break_val <= 0:
                break_val = 1
        elif cur_speed >= 100:
            print('===========break_C')
            break_val = input_speed + 20 - int(0.3*cur_speed)
            if break_val <= 0:
                break_val = 1
    elif cur_mode== 'ctrl_E_break':
        speed=0
        break_val = 0xff
    elif cur_mode== 'ctrl_break_crosswalk':
        action_break_crosswalk()
        
    elif cur_mode== 'ctrl_chng_right':
        msg_sync.gps_mission_index = 0
        msg_sync.ctrl_mode = 'ctrl_normal'
        pub.publish(msg_sync)
        action_lane_right()
        
    elif cur_mode== 'ctrl_chng_left':
        msg_sync.gps_mission_index = 0
        msg_sync.ctrl_mode = 'ctrl_normal'
        pub.publish(msg_sync)
        action_lane_left()
        
    elif cur_mode== 'ctrl_parking':
        msg_sync.gps_mission_index = 0
        msg_sync.ctrl_mode = 'ctrl_normal'
        pub.publish(msg_sync)
        action_parking()
        
    
    # steering 값 2000 넘길 시 2000으로 설정
    if abs(steering)>2000:
        if steering>0:
            steering = 2000
        else :
            steering =-2000
    # if abs(steering) < 300 :
    #     steering = int(0.5*steering)

    # 기어 기본값 0: 전진, 1:후진
    print('speed : ', speed);
    print("OUT_PUT | speed : ", speed, ', steering : ',steering, ', break : ', break_val)
    print('OUT_PUT | cnt : ', cnt, ', MODE : ' , cur_mode)
    result = struct.pack('!BBBBBBHhBBBB', 0x53, 0x54, 0x58, 0x01, 0x00, 0x00, int(speed),
                steering, break_val, cnt, 0x0D, 0x0A )    # big endian 방식으로 타입에 맞춰서 pack   
    # print("pc : ", result[0], result[1], result[2], result[3], result[4], result[5], result[6], result[7], result[8],
    #   result[9], result[10], result[11], result[12], result[13] )
    ser.write(result)
    msg_sync.erp_speed         = int(speed/10)
    msg_sync.erp_steering      = int(steering/71)
    msg_sync.erp_break         = int(break_val)
    msg_sync.gps_mission_index = int(path_type[path_idx])
    msg_sync.ctrl_mode = 'ctrl_normal'
    pub.publish(msg_sync)
    # print('========publish==============')
    # print(msg_sync)

    
    
def Y_angle(cur_x, cur_y, cur_yaw, target_x, target_y): # By 삼각함수 멋쟁이 예환
    msg_sync.target_x = int(target_x)
    msg_sync.target_y = int(target_y)
    # print("Y_angle", cur_x, cur_y, cur_yaw, target_x, target_y)
    yaw=radians(cur_yaw)
    # print(degrees(atan2((target_y - cur_y), (target_x - cur_x))))
    ######################## 원래 1번째 줄 코드였는데... -였는데 +로 바꿈
    # th = cur_yaw + degrees(atan2((target_y - cur_y), (target_x - cur_x)))
    th = (cur_yaw + degrees(atan2((target_y - cur_y), (target_x - cur_x))))
    # J_th
    J_th = degrees(atan2(2*1.1*sin(radians(th))/(look_ahead/10),1))
    if abs(J_th)>180:
        if (J_th < 0) :
            J_th += 360
        else :
            J_th -= 360

    print("Theta : ", J_th)
    if abs(J_th)>30:
        if J_th > 0:
            return 30
        else :
            return -30
    else :
        return int(J_th)


def cal_steering(cur_x, cur_y, cur_yaw, look_ahead):
    print("cal_steering", cur_x, cur_y, cur_yaw, look_ahead)    
    global path_idx
    print("path_idx", path_idx)    
    target_x = path_x[path_idx]
    target_y = path_y[path_idx]
    dis = pow(pow(abs(target_x - cur_x),2) + pow(abs(target_y - cur_y),2), 0.5)
    # print("distance : ", dis)
    tmp_look_ahead = look_ahead
    if path_k[path_idx] != 0 :
        tmp_look_ahead = 30
    print('tmp_look_ahead :', tmp_look_ahead)
    if dis <= tmp_look_ahead:
        print("### HIT target!!! ###")
        path_idx += 1
        return cal_steering(cur_x, cur_y, cur_yaw, tmp_look_ahead)
    else : 
         return Y_angle(cur_x, cur_y, cur_yaw, target_x, target_y)

def cal_lookahead(vel):
    # print('vel : ', vel)
    look_ahead = k*int(vel)/3.6 + 20
    # look_ahead = 30  #3.3?
    return look_ahead

# msg 수신시 호출되는 함수
def getSyncMsg(msg):
    global first_chk
    global path_idx
    global look_ahead
    global cur_speed
    print("======== get a odo msg =========")
    # print(msg)
    # cur_x    = msg.gps_x/2
    # cur_y    = msg.gps_y/2
    cur_x    = msg.gps_x
    cur_y    = msg.gps_y
    cur_yaw  = msg.imu_yaw
    cur_mode = msg.ctrl_mode
    
    
    if(first_chk):
        find_first_pathidx(cur_x, cur_y, cur_yaw)

    if cur_yaw > 90 and cur_yaw < 270:
        cur_yaw = -(cur_yaw-90);
    else :
        if cur_yaw < 90:
            cur_yaw = (90-cur_yaw);
        else :
            cur_yaw = 360-cur_yaw +90     

    cnt=0x00;
    if cur_mode == 'ctrl_normal' :
        if path_idx != len(path_k):
            if path_k[path_idx] != 0 :
                cur_mode = 'ctrl_slow_1'
            else : 
                cur_mode = 'ctrl_normal'
        if path_type[path_idx] != 0 :
            cur_mode = 'ctrl_slow_2'

    steering = cal_steering(cur_x, cur_y, cur_yaw, look_ahead)
    result = ser.readline() # erp -> pc
    ser.flushInput()
    # print(result)   

    print("len", len(result)) 
    if len(result) > 17:
        # print("erp : ", result[0], result[1], result[2], result[3], result[4], result[5], result[6], result[7], result[8],
        # result[9], result[10], result[11], result[12], result[13], result[14], result[15], result[16], result[17])
        cnt = result[15]
        print(cnt)
        # look_ahead = cal_lookahead(result[6])
        steering = cal_steering(cur_x, cur_y, cur_yaw, look_ahead);
        cur_speed = int(result[6])
        serWrite(int(speed_lim*10), int(steering*71), cnt, cur_mode, cur_speed)
        print("cur_speed : " , result[6])
        
    # cnt가 10일때 패킷이 잘려서 2번에 걸쳐 들어옴.+++++++++++++++++++++++++++++ 0x0a값이 아스키코드 LF(new line)!!!
    elif len(result) == 16:
        # print("erp : ", result[0], result[1], result[2], result[3], result[4], result[5], result[6], result[7], result[8],
        # result[9], result[10], result[11], result[12], result[13], result[14], result[15])
        add_result = ser.readline() # erp -> pc
        print(cnt)        
        # look_ahead = cal_lookahead(result[6])
        steering = cal_steering(cur_x, cur_y, cur_yaw, look_ahead);
        print("cur_speed : " , result[6])
        cur_speed = int(result[6])
        serWrite(int(speed_lim*10), int(steering*71), 10, cur_mode, cur_speed)
        
    # serWrite(int(speed_lim*10), int(steering*71), 10, cur_mode)
    
    


def find_first_pathidx(cur_x, cur_y, cur_yaw):
    global path_idx
    global first_chk
    min_idx = 0
    min_dis = 9999999999
    size = len(path_x)
    for i in range(size):
        x = path_x[i]
        y = path_y[i]
        dis = (x-cur_x)**2 + (y-cur_y)**2
        if(min_dis > dis):
            min_dis = dis
            min_idx = i
    print("fin_dis : ", min_dis)
    print("fin_idx : ", min_idx)
    path_idx = min_idx
    first_chk = False
    yaw = Y_angle(cur_x, cur_y, cur_yaw, path_x[min_idx], path_y[min_idx])
    if(abs(yaw)>90):
        path_idx += 1



def main(): 
    # with open('/home/wego/catkin_ws/src/master/src/waypoint/songdo2_50cm.csv', mode='r') as csv_file:
    # with open('/home/wego/catkin_ws/src/master/src/waypoint/songdo_map_50cm.csv', mode='r') as csv_file:
    # with open('/home/wego/catkin_ws/src/master/src/waypoint/square_1m.csv', mode='r') as csv_file:
        
    with open('/home/wego/catkin_ws/src/master/src/waypoint/k-city_50cm_type.csv', mode='r') as csv_file:
        csv_reader = csv.DictReader(csv_file)
        for next_r in csv_reader:
            path_x.append(float(next_r['X']))
            path_y.append(float(next_r['Y']))
            path_k.append(float(next_r['K']))
            path_type.append(float(next_r['type']))
            print("local_x ",float(next_r['X']))
            print("local_y ",float(next_r['Y']))

    rospy.init_node('control_node', disable_signals=False)
    
    rospy.loginfo("-------control node start!-------")
    # rospy.Subscriber("/master", sync_msg, getSyncMsg)
    rospy.loginfo("-------start spin-------")
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        msg = rospy.wait_for_message("/master", sync_msg, timeout=None)
        getSyncMsg(msg)
        rate.sleep()
    # rospy.spin()
    rospy.loginfo("-------finish spin-------")
    
 
if __name__ == '__main__':
    print("Pure pursuit path tracking simulation start")
    main()