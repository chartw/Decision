#!/usr/bin/env python3

import rospy
import pymap3d
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from sensor_msgs.msg import ChannelFloat32
import pcl
#import pcl_helper
import numpy as np
import numpy as np
from nav_msgs.msg import Odometry
from math import cos, sin
from shapely.geometry import Point, Polygon
from std_msgs.msg import Int32, String

global lidar_temp, lidar_cur_state
lidar_temp = PointCloud2()
lidar_cur_state = 'static'

#downsampling
def do_passthrough(pcl_data,filter_axis,axis_min,axis_max):
    passthrough = pcl_data.make_passthrough_filter()
    passthrough.set_filter_field_name(filter_axis)
    passthrough.set_filter_limits(axis_min, axis_max)
    return passthrough.filter()

# ROI
def do_voxel_grid_downssampling(pcl_data,leaf_size):
    vox = pcl_data.make_voxel_grid_filter()
    vox.set_leaf_size(leaf_size, leaf_size, leaf_size) # The bigger the leaf size the less information retained
    return  vox.filter()

def getcurstate(msg):
    global lidar_cur_state
    lidar_cur_state = msg

def getMsg_lidar(msg):
    global lidar_temp
    lidar_temp = msg

# No mission part------------------------------------------------
def getMsg_non():
    test = PointCloud()
    pub.publish(test)

# static obstacle part------------------------------------------------
def getMsg_static(lidar_data):
    gen = point_cloud2.read_points(lidar_data, skip_nans=True)
    points_list = []

    for p in gen:
        if p[4]==7:
            points_list.append([p[0], p[1], p[2], p[3]])

    pcl_data = pcl.PointCloud_PointXYZRGB()
    pcl_data.from_list(points_list)

    # downsampling 실행 코드 부분
    print("Input :", pcl_data)

    LEAF_SIZE = 0.01
    cloud = do_voxel_grid_downssampling(pcl_data, LEAF_SIZE)
    print("Output :", cloud)

    # ROI 실행 코드 부분 
    filter_axis = 'x'
    axis_min = 0
    axis_max = 10.0
    cloud = do_passthrough(cloud, filter_axis, axis_min, axis_max)

    filter_axis = 'y'
    axis_min = -3.0
    axis_max = 3.0
    cloud = do_passthrough(cloud, filter_axis, axis_min, axis_max)

    filter_axis = 'z'
    axis_min = -0.1
    axis_max = 2
    cloud = do_passthrough(cloud, filter_axis, axis_min, axis_max)


    test = PointCloud()
    get_in = ChannelFloat32()
    get_in.name = 'VLP_intensity'
    test.points = []
    for p in cloud:
        park = Point32()
        park.x = p[0]
        park.y = p[1]
        park.z = 0
        get_in.values.append(p[3])
        test.points.append(park)

    test.channels.append(get_in)
    test.header.frame_id = 'world'
    test.header.stamp = rospy.Time.now()
    pub.publish(test)

# dynamic obstacle part------------------------------------------------
def getMsg_dynamic(lidar_data):

    gen = point_cloud2.read_points(lidar_data, skip_nans=True)
    points_list = []

    for p in gen:
        points_list.append([p[0], p[1], p[2], p[3]])

    pcl_data = pcl.PointCloud_PointXYZRGB()
    pcl_data.from_list(points_list)
   
    # downsampling 실행 코드 부분
    print("Input :", pcl_data)

    LEAF_SIZE = 0.1
    cloud = do_voxel_grid_downssampling(pcl_data, LEAF_SIZE)
    print("Output :", cloud)

    # ROI 실행 코드 부분 
    filter_axis = 'x'
    axis_min = 1.0
    axis_max = 3.0
    cloud = do_passthrough(cloud, filter_axis, axis_min, axis_max)

    filter_axis = 'y'
    axis_min = -4.0
    axis_max = 4.0
    cloud = do_passthrough(cloud, filter_axis, axis_min, axis_max)

    filter_axis = 'z'
    axis_min = -4.0
    axis_max = 2.0
    cloud = do_passthrough(cloud, filter_axis, axis_min, axis_max)

    test = PointCloud()
    get_in = ChannelFloat32()
    get_in.name = 'VLP_intensity'
    test.points = []
    for p in cloud:
        # if p[1] > 0:
        park = Point32()
        park.x = p[0]
        park.y = p[1]
        park.z = 0
        get_in.values.append(p[3])
        test.points.append(park)

    #print("Input :", cnt)
    test.channels.append(get_in)
    test.header.frame_id = 'world'
    test.header.stamp = rospy.Time.now()
    pub.publish(test)


# Parking part------------------------------------------------
global yaw, cur_x, cur_y, flag
parking_state = True
pi = 3.1415926535897931
yaw = 0
cur_x = 0
cur_y = 0
flag = False

def getodo(msg):
    global yaw, cur_x, cur_y, flag
    yaw = msg.twist.twist.angular.z
    cur_x = msg.pose.pose.position.x
    cur_y = msg.pose.pose.position.y

def get_xy(lat, lon, base_lat, base_lon):
    e, n, u = pymap3d.geodetic2enu(lat, lon, 15.400, base_lat, base_lon, 15.400)
    return e, n

def parking(temp_points):
	# 좌표 입력 부분
    base1_lat = 37.383784
    base1_lon = 126.654310 

    pp = [[37.38416, 126.654545],[37.384171, 126.654493],[37.384139, 126.654525],[37.38415, 126.654473],[37.384118, 126.654505],[37.38413, 126.654454],[37.384099, 126.654486],[37.384108, 126.654433],[37.384079, 126.654467],[37.384088, 126.654414],[37.384059, 126.654448],[37.384067, 126.654394]]

    parking_point_x_y = []

    for i in range(len(pp)):
        parking_point_x_y.append(get_xy(pp[i][0], pp[i][1], base1_lat, base1_lon))

    print(parking_point_x_y)

    parking_space_1 = [parking_point_x_y[0], parking_point_x_y[1], parking_point_x_y[3], parking_point_x_y[2]]
    parking_space_2 = [parking_point_x_y[2], parking_point_x_y[3], parking_point_x_y[5], parking_point_x_y[4]]
    parking_space_3 = [parking_point_x_y[4], parking_point_x_y[5], parking_point_x_y[7], parking_point_x_y[6]]
    parking_space_4 = [parking_point_x_y[6], parking_point_x_y[7], parking_point_x_y[9], parking_point_x_y[8]]
    parking_space_5 = [parking_point_x_y[8], parking_point_x_y[9], parking_point_x_y[11], parking_point_x_y[10]]
    # 직사각형 생성
    parking_space_poly1 = Polygon(parking_space_1)
    parking_space_poly2 = Polygon(parking_space_2)
    parking_space_poly3 = Polygon(parking_space_3)
    parking_space_poly4 = Polygon(parking_space_4)
    parking_space_poly5 = Polygon(parking_space_5)


    parking_result = [0, 0, 0, 0, 0]
    for i in range(len(temp_points)):
        test_code = Point(temp_points[i].x, temp_points[i].y)
        if test_code.within(parking_space_poly1):
            parking_result[0]+=1
        if test_code.within(parking_space_poly2):
            parking_result[1]+=1
        if test_code.within(parking_space_poly3):
            parking_result[2]+=1
        if test_code.within(parking_space_poly4):
            parking_result[3]+=1
        if test_code.within(parking_space_poly5):
            parking_result[4]+=1

    print("parking 1 :", parking_result[0]) 
    print("parking 2 :", parking_result[1])
    print("parking 3 :", parking_result[2])
    print("parking 4 :", parking_result[3])
    print("parking 5 :", parking_result[4])
    
    result_number = -1
    for i in range(0, 5):
        if parking_result[i] < 10:
            result_number = i + 1
            return result_number
    return result_number


def getMsg_parking(lidar_data):
    global yaw, cur_x, cur_y, flag

    gen = point_cloud2.read_points(lidar_data, skip_nans=True)
    cnt = 0
    points_list = []

    for p in gen:
        points_list.append([p[0], p[1], p[2], p[3]])

    pcl_data = pcl.PointCloud_PointXYZRGB()
    pcl_data.from_list(points_list)

    print("Input :", pcl_data)

    LEAF_SIZE = 0.03
    cloud = do_voxel_grid_downssampling(pcl_data, LEAF_SIZE)
    
    print("")

    filter_axis = 'x'
    axis_min = 0
    axis_max = 20
    cloud = do_passthrough(cloud, filter_axis, axis_min, axis_max)

    filter_axis = 'y'
    axis_min = -20
    axis_max = 20
    cloud = do_passthrough(cloud, filter_axis, axis_min, axis_max)

    filter_axis = 'z'
    axis_min = -0.2
    axis_max = 2

    cloud = do_passthrough(cloud, filter_axis, axis_min, axis_max)
    print("Output :", cloud)

    test = PointCloud()
    get_in = ChannelFloat32()
    get_in.name = 'VLP_intensery'
    test.points = []
    theta = (yaw) * pi / 180;
    for p in cloud:
        park = Point32()
        park.x = p[0] * cos(theta) + p[1] * -sin(theta) + cur_x
        park.y = p[0] * sin(theta) + p[1] * cos(theta) + cur_y
        park.z = 0
        get_in.values.append(p[3])
        test.points.append(park)
        cnt += 1

    
    parking_number = Int32()
    print(type(test.points))
    parking_number.data = parking(test.points)
    pub_num.publish(parking_number)

    #print("Input :", cnt)
    test.channels.append(get_in)
    test.header.frame_id = 'world'
    test.header.stamp = rospy.Time.now()
    pub.publish(test)


# # 뭐가 on 되는지 확인하는 용도로 있으면 좋을듯
def Do_state():
    global lidar_cur_state, lidar_temp
    if lidar_cur_state == 'non':
        print("lidar_non")
        getMsg_non()
    elif lidar_cur_state == 'static':
        print("static ON")
        getMsg_static(lidar_temp)
    elif lidar_cur_state == 'dynamic':
        print("dynamic ON")
        getMsg_dynamic(lidar_temp)
    elif lidar_cur_state == 'parking':
        print("parking ON")
        getMsg_parking(lidar_temp)

rospy.init_node("lidar", anonymous=True)
pub = rospy.Publisher("lidar_state_publish", PointCloud, queue_size =10)
pub_num = rospy.Publisher("Parking_num", Int32, queue_size =10)

while not rospy.is_shutdown():
    rospy.Subscriber("/pose", Odometry, getodo)
    rospy.Subscriber("/cur_state", String, getcurstate)
    rospy.Subscriber("/velodyne_points", PointCloud2, getMsg_lidar)
    Do_state()
    # rospy.spin()