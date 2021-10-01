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
from master_node.msg import Planning_Info

global lidar_temp, lidar_cur_state
lidar_temp = PointCloud2()
lidar_cur_state = 'centroid'

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

#planning
def getplan(msg):
    global lidar_cur_state
    lidar_cur_state = msg.mode


def getMsg_lidar(msg):
    global lidar_temp
    lidar_temp = msg

def getMsg_centroid(lidar_data):
    gen = point_cloud2.read_points(lidar_data, skip_nans=True)
    points_list = []

    for p in gen:
        # if p[4]==7:
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
    axis_min = 0.2
    axis_max = 10.0
    cloud = do_passthrough(cloud, filter_axis, axis_min, axis_max)

    filter_axis = 'y'
    axis_min = -3.0
    axis_max = 0.0
    cloud = do_passthrough(cloud, filter_axis, axis_min, axis_max)

    filter_axis = 'z'
    axis_min = -0.1
    axis_max = 3.0
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
        park.z = p[2]
        get_in.values.append(p[3])
        test.points.append(park)

    #print("Input :", cnt)
    test.channels.append(get_in)
    test.header.frame_id = 'world'
    test.header.stamp = rospy.Time.now()
    pub.publish(test)

# # 뭐가 on 되는지 확인하는 용도로 있으면 좋을듯
def Do_state():
    global lidar_cur_state, lidar_temp
    getMsg_centroid(lidar_temp)
    print(lidar_cur_state)

rospy.init_node("lidar", anonymous=True)
pub = rospy.Publisher("lidar_pub", PointCloud, queue_size=1)
pub_num = rospy.Publisher("Parking_num", Int32, queue_size=1)
rospy.Subscriber("/planner", Planning_Info, getplan)
rospy.Subscriber("/velodyne_points", PointCloud2, getMsg_lidar)

rate = rospy.Rate(20)  # 100hz

while not rospy.is_shutdown():

    Do_state()
    rate.sleep()

    # rospy.spin()
