#!/usr/bin/env python
import rospy
import numpy as np
import csv
import cv2
from math import radians
from math import cos
from math import sin
from master.msg import sync_msg
from time import sleep

base_lat = 37.3837028
base_lon = 126.653281
# base_lat = 37.237898722
# base_lon = 126.77174665

map_img_loaded = cv2.imread('/home/wego/catkin_ws/src/master/src/map/songdo_map_.jpg', 3)
# map_img_loaded = cv2.imread('/home/wego/catkin_ws/src/master/src/map/k-city_400_990.jpg', 3)
# map_big_img_loaded = cv2.imread('/home/wego/catkin_ws/src/master/src/map/k-city_4000_9900.jpg', 3)
map_height = 1800
map_width = 3500
# map_height = 9900
# map_width = 4000
# map_global = np.zeros([global_height, global_width], dtype=int)  # Global grid
path_idx = 0


mode_cross_straight = False
mode_cross_right    = False
mode_cross_left     = False
mode_cross_right_no = False
mode_cross_left_no  = False
mode_object_static  = False 
mode_object_dynamic = False
mode_crosswalk      = False
mode_parking        = False
mode_school_zone    = False 
mode_bump           = False


def draw_path():
    # with open('/home/wego/catkin_ws/src/master/src/waypoint/k-city_interval_7m.csv', mode='r') as csv_file:
    # with open('/home/wego/catkin_ws/src/master/src/waypoint/songdo_curveL1.csv', mode='r') as csv_file:
    with open('/home/wego/catkin_ws/src/master/src/waypoint/songdo_maptest.csv', mode='r') as csv_file:
    # with open('/home/wego/catkin_ws/src/master/src/waypoint/songdo_map_50cm.csv', mode='r') as csv_file:
    
    # with open('/home/wego/catkin_ws/src/master/src/waypoint/square_1m.csv', mode='r') as csv_file:
        
        csv_reader = csv.DictReader(csv_file)
        for next_r  in csv_reader:
            cv2.line(map_img_loaded, (int(float(next_r['X'])), int(float(next_r['Y']))), (int(float(next_r['X'])), int(float(next_r['Y']))), (255, 0, 250), 3)
            # cv2.line(map_img_loaded, (int(float(next_r['X'])), int(float(next_r['Y']))), (int(float(next_r['X'])), int(float(next_r['Y']))), (255, 0, 250), 3)
            # cv2.line(map_big_img_loaded, (int(float(next_r['X'])), int(float(next_r['Y']))), (int(float(next_r['X'])), int(float(next_r['Y']))), (255, 0, 250), 3)

            # if int(next_r['type']) == 0 : 
            #     cv2.line(map_img_loaded, (int(float(next_r['X'])/10), int(float(next_r['Y'])/10)), (int(float(next_r['X'])/10), int(float(next_r['Y'])/10)), (255, 0, 250), 3)
            #     cv2.line(map_big_img_loaded, (int(float(next_r['X'])), int(float(next_r['Y']))), (int(float(next_r['X'])), int(float(next_r['Y']))), (255, 0, 250), 3)
            # else : 
            #     cv2.line(map_img_loaded, (int(float(next_r['X'])/10), int(float(next_r['Y'])/10)), (int(float(next_r['X'])/10), int(float(next_r['Y'])/10)), (0, 0, 255), 3)
            #     cv2.line(map_big_img_loaded, (int(float(next_r['X'])), int(float(next_r['Y']))), (int(float(next_r['X'])), int(float(next_r['Y']))), (0, 0, 255), 3)


def draw_map(x, y, target_x, target_y, imu_yaw):
    draw_path()
    # cv2.line(map_img_loaded, (int(x/10), int(y/10)), (int(x/10), int(y/10)), (255, 250, 0), 3)
    cv2.line(map_img_loaded, (int(x/2), int(y/2)), (int(x/2), int(y/2)), (255, 250, 0), 3)
    # dir_map = map_big_img_loaded
    dir_map = map_img_loaded
    
    if target_x != 0 :
        cv2.line(dir_map, (int(x/2), int(y/2)), (int(target_x), int(target_y)), (255, 250, 255), 1)
        # cv2.line(map_big_img_loaded, (int(x), int(y)), (int(target_x), int(target_y)), (255, 250, 255), 1)

    if imu_yaw > 90 and imu_yaw < 270:
        imu_yaw = -(imu_yaw-90);
    else :
        if imu_yaw < 90:
            imu_yaw = (90-imu_yaw);
        else :
            imu_yaw = 360-imu_yaw +90     

    cv2.line(map_img_loaded, (int(x/2), int(y/2)), (int(50*cos(radians(imu_yaw))) + int(x/2), -int(50*sin(radians(imu_yaw))) + int(y/2)), (0, 0, 255), 1)
    # cv2.line(map_big_img_loaded, (int(x), int(y)), (int(50*cos(radians(imu_yaw))) + int(x), -int(50*sin(radians(imu_yaw))) + int(y)), (0, 0, 255), 1)
    cv2.imshow('global_map', map_img_loaded)
# sz    cv2.imshow('dir_map', dir_map)

    # cv2.line(map_big_img_loaded, (x, y), (x, y), (255, 250, 0), 3)
    # map_big_loaded = map_big_img_loaded
    # map_big_loaded = map_big_loaded[y-300:y+300, x-300:x+300]
    # map_big_loaded = cv2.resize(map_big_loaded, dsize=(700, 700), interpolation=cv2.INTER_AREA)
    # cv2.imshow('big_map', map_big_loaded)

    cv2.waitKey(1)
    

def local_change(x, y):
    print(x, y)
    cx = int((float(x) - float(base_lon)) * 1000000)
    # cy = map_height - int((float(y) - float(base_lat)) * 1000000)
    cy = map_height - int((float(y) - float(base_lat)) * 1000000)
    print(cx, cy)
    return (cx, cy)
    # return (cx/2, cy/2)


def clear_mode():
    global mode_cross_straight 
    global mode_cross_right    
    global mode_cross_left     
    global mode_cross_right_no
    global mode_cross_left_no
    global mode_object_static  
    global mode_object_dynamic 
    global mode_crosswalk      
    global mode_parking        
    global mode_school_zone    
    global mode_bump
    mode_cross_straight = False
    mode_cross_right    = False
    mode_cross_left     = False
    mode_cross_right_no = False
    mode_cross_left_no  = False
    mode_object_static  = False 
    mode_object_dynamic = False
    mode_crosswalk      = False
    mode_parking        = False
    mode_school_zone    = False 
    mode_bump           = False


def msgCallback(msg):
    global mode_cross_straight 
    global mode_cross_right    
    global mode_cross_left     
    global mode_cross_right_no
    global mode_cross_left_no
    global mode_object_static  
    global mode_object_dynamic 
    global mode_crosswalk      
    global mode_parking        
    global mode_school_zone    
    global mode_bump 
    print('*****get message*****')          
    print(msg)
    msg.gps_x, msg.gps_y = local_change(msg.gps_x, msg.gps_y)
    draw_map(msg.gps_x, msg.gps_y, msg.target_x, msg.target_y, msg.imu_yaw)
    gps_mission_index = msg.gps_mission_index

    
    if gps_mission_index == 1 :
        if msg.lane == 'lane_stop' and msg.sign == 'sign_traffic_red':
            print('get stop!! ')        
            msg.ctrl_mode = 'ctrl_break'
        elif msg.lane == 'lane_stop_close' and msg.sign == 'sign_traffic_red':
            print('get stop!! ')        
            msg.ctrl_mode = 'ctrl_break'
        else :
            msg.ctrl_mode = 'ctrl_normal'
    # #-----------------------------------------#
    # if msg.sign == 'sign_empty':
    #     pass
    # elif msg.sign == 'sign_right':
    #     mode_cross_right = True
    # elif msg.sign == 'sign_left':
    #     mode_cross_left = True
    # #-----------------------------------------#
    # if mode_cross_left and msg.lane == 'lane_stop':
    #     msg.ctrl_mode = 'ctrl_slow_3'
    # elif mode_cross_left and msg.lane == 'lane_stop_close':
    #     msg.ctrl_mode = 'ctrl_break'

    # if mode_cross_left and msg.sign == 'sign_traffic_left':
    #     clear_mode()
    #     msg.ctrl_mode = 'ctrl_normal'
    # #-----------------------------------------#
    talker(msg)
        

def listner():
    rospy.init_node('master_node')
    rospy.loginfo("-------master_node start!-------")
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        msg = rospy.wait_for_message("/sync", sync_msg, timeout=None)
        msgCallback(msg)
        rate.sleep()
                

def talker(msg):
    pub = rospy.Publisher('/master', sync_msg, queue_size = 10)
    pub.publish(msg)
    

if __name__ == "__main__":
    #make_map()
    draw_path()
    listner()
    cv2.destroyAllWindows()
