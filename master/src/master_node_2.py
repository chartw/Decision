#!/usr/bin/env python
import rospy
import numpy as np
import csv
import cv2
from time import sleep
from master.msg import sync_msg

base_lat = 37.3837028
base_lon = 126.653281
# base_lat = 37.237898722
# base_lon = 126.77174665

map_img_loaded = cv2.imread('/home/wego/catkin_ws/src/master/src/map/songdo_map_.jpg', 3)
# map_img_loaded = cv2.imread('/home/wego/catkin_ws/src/master/src/map/k-city.jpg', 3)
global_height = 1800
global_width = 3500
# global_height = 1980
# global_width = 800
map_global = np.zeros([global_height, global_width], dtype=int)  # Global grid

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
lidar_cnt = 0 

def draw_path():
    with open('/home/wego/catkin_ws/src/master/src/waypoint/testmap.csv', mode='r') as csv_file:
        csv_reader = csv.DictReader(csv_file)
        for next_r  in csv_reader:
            cv2.line(map_img_loaded, (int(float(next_r['X'])/2), int(float(next_r['Y'])/2)), (int(float(next_r['X'])/2), int(float(next_r['Y'])/2)), (255, 0, 250), 3)

            
def make_map():
    with open('/home/wego/catkin_ws/src/master/src/map/songdo_map_.csv', mode='r') as csv_file:
    # with open('/home/wego/catkin_ws/src/master/src/map/k-city.csv', mode='r') as csv_file:
        csv_reader = csv.DictReader(csv_file)
        for next_r in csv_reader:
            dy = int((float(next_r['Y']) - float(base_lat)) * 1000000)
            dx = int((float(next_r['X']) - float(base_lon)) * 1000000)
            map_global[dy, dx] = 1
    

def draw_map(x, y):
    cv2.line(map_img_loaded, (int(x/2), int(y/2)), (int(x/2), int(y/2)), (255, 250, 0), 3)
    map_loaded = cv2.flip(map_img_loaded, 0)
    cv2.imshow('global_map', map_loaded)
    cv2.waitKey(1)
    

def local_change(x, y):
    cx = int((float(x) - float(base_lon)) * 1000000)
    cy = int((float(y) - float(base_lat)) * 1000000)
    return (cx, cy)


def clear_mode():
    global mode_cross_straight 
    global mode_cross_right
    global mode_cross_left     
    global mode_cross_right_no
    global mode_cross_left_no
    global mode_object_static  
    global mode_object_dynamic 
    global mode_crosswalk
    global mode_school_zone    
    global mode_bump
    global mode_parking

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
    global mode_school_zone    
    global mode_bump
    global mode_parking
    global lidar_cnt
    msg.ctrl_mode = 'ctrl_normal'
    msg.gps_x, msg.gps_y = local_change(msg.gps_x, msg.gps_y)
    draw_map(msg.gps_x, msg.gps_y)

    #------------------trigger------------------#
    if msg.sign == 'sign_empty':
        msg.ctrl_mode = 'ctrl_normal'
    elif msg.sign == 'sign_cross':
        mode_cross_straight = True
    elif msg.sign == 'sign_right':
        mode_cross_right = True
    elif msg.sign == 'sign_left':
        mode_cross_left = True
    elif msg.sign == 'sign_object_static':
        mode_object_static = True
    elif msg.sign == 'sign_object_dynamic':
        mode_object_dynamic = True
    elif msg.sign == 'sign_crosswalk':
        mode_crosswalk = True
    elif msg.sign == 'sign_school_zone' :
        mode_school_zone = True
    elif msg.sign == 'sign_bump' :
        mode_bump = True
    elif msg.sign == 'sign_parking':
        mode_parking = True
    
        

    if msg.lane == 'lane_school_zone':
        mode_school_zone = True
    elif msg.lane == 'lane_bump':
        mode_bump = True

    #---------------mode_cross_straight-------------#
    if mode_cross_straight and msg.lane == 'lane_stop':
        msg.ctrl_mode = 'ctrl_slow_3'
    elif mode_cross_straight and msg.lane == 'lane_stop_close':
        msg.ctrl_mode = 'ctrl_break' 

    if mode_cross_straight and msg.sign == 'sign_traffic_green': #멀리서 잡는다면 안댐.
        msg.ctrl_mode = 'ctrl_normal'
        print('mode_cross_straight_finish')
        clear_mode()
    #---------------mode_cross_left-----------------#
    if mode_cross_left and msg.lane == 'lane_stop':
        msg.ctrl_mode = 'ctrl_slow_3'
    elif mode_cross_left and msg.lane == 'lane_stop_close':
        msg.ctrl_mode = 'ctrl_break'

    if mode_cross_left and msg.sign == 'sign_traffic_left':
        clear_mode()
        msg.ctrl_mode = 'ctrl_normal'
    # ---------------mode_cross_right----------------#
    if mode_cross_right :
        msg.ctrl_mode = 'ctrl_slow_3'
    elif mode_cross_right  and msg.lane == 'lane_stop_close':
        msg.ctrl_mode = 'ctrl_break'

    if mode_cross_left and msg.sign == 'sign_traffic_left':
        clear_mode()
        msg.ctrl_mode = 'ctrl_normal'
    # --------------mode_object_static---------------#
    if mode_object_static and msg.lidar == 'lidar_front_3_5m' and lidar_cnt == 1:
        msg.ctrl_mode = 'ctrl_chng_left'
        lidar_cnt = 0
        clear_mode()
    elif mode_object_static and msg.lidar == 'lidar_front_3_5m':
        msg.ctrl_mode = 'ctrl_chng_right'
        lidar_cnt = 1
    elif mode_object_static:
        msg.ctrl_mode = 'ctrl_slow_6'
    # --------------mode_object_dynamic-------------#
    if mode_object_dynamic:
        msg.ctrl_mode = 'ctrl_slow_3'
    elif mode_object_dynamic and msg.lidar == 'lidar_front_3_5m':
        msg.ctrl_mode = 'ctrl_break'
    elif mode_object_dynamic and msg.lidar == 'lidar_empty':
        clear_mode()
    # --------------mode_crosswalk------------------#
    if mode_crosswalk:
        msg.ctrl_mode = 'ctrl_slow_3'
    elif mode_crosswalk and msg.lane == 'lane_stop':
        msg.ctrl_mode = 'ctrl_slow_3'
    elif mode_crosswalk and msg.lane == 'lane_stop_close':
        msg.ctrl_mode = 'ctrl_break_crosswalk'
        clear_mode()
    # --------------mode_school_zone----------------#
    if mode_school_zone:
        msg.ctrl_mode = 'ctrl_slow_6'
        clear_mode()
    # --------------mode_bump-----------------------#
    if mode_bump:
        msg.ctrl_mode = 'ctrl_slow_3'
        clear_mode()
    #-------------------parking-----------------#
    if mode_parking :
        msg.ctrl_mode = 'ctrl_slow_3'
    elif mode_parking and msg.lidar2 == 'lidar_right_empty':
        msg.ctrl_mode = 'ctrl_parking'
        clear_mode()
    #-------------------------------------------#


    
    print(msg.ctrl_mode)
    if msg.ctrl_mode == 'ctrl_chng_right' :
        sleep(5)
    talker(msg)
    print('----------------------------------')
    

def listner():
    rospy.init_node('master_node')
    rospy.loginfo("-------master_node start!-------")
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        msg = rospy.wait_for_message("/sync", sync_msg, timeout=None)
        msgCallback(msg)
        try:
            talker(msg)
            rate.sleep()
        except rospy.ROSInterruptException:pass
        

def talker(msg):
    pub = rospy.Publisher('/master', sync_msg, queue_size = 10)
    pub.publish(msg)
    

if __name__ == "__main__":
    #make_map()
    draw_path()
    listner()
    cv2.destroyAllWindows()
