#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from sensor_msgs.msg import ChannelFloat32
import pcl
#import pcl_helper
import numpy as np



def do_passthrough(pcl_data,filter_axis,axis_min,axis_max):
    '''
    Create a PassThrough  object and assigns a filter axis and range.
    :param pcl_data: point could data subscriber
    :param filter_axis: filter axis
    :param axis_min: Minimum  axis to the passthrough filter object
    :param axis_max: Maximum axis to the passthrough filter object
    :return: passthrough on point cloud
    '''
    passthrough = pcl_data.make_passthrough_filter()
    passthrough.set_filter_field_name(filter_axis)
    passthrough.set_filter_limits(axis_min, axis_max)
    return passthrough.filter()


def do_voxel_grid_downssampling(pcl_data,leaf_size):
    ''' 
    Create a VoxelGrid filter object for a input point cloud
    :param pcl_data: point cloud data subscriber
    :param leaf_size: voxel(or leaf) size
    :return: Voxel grid downsampling on point cloud
    :https://github.com/fouliex/RoboticPerception
    '''
    vox = pcl_data.make_voxel_grid_filter()
    vox.set_leaf_size(leaf_size, leaf_size, leaf_size) # The bigger the leaf size the less information retained
    return  vox.filter()


def getMsg(msg):

    gen = point_cloud2.read_points(msg, skip_nans=True)   
    # print(type(gen))
    cnt = 0
    points_list = []

    for p in gen:
        # if p[4] is 7:
        points_list.append([p[0], p[1], p[2], p[3]])

    pcl_data = pcl.PointCloud_PointXYZRGB()
    #pcl_data = points_list
    pcl_data.from_list(points_list)


   
    # downsampling 실행 코드 부분
    print("Input :", pcl_data)

    LEAF_SIZE = 0.1
    cloud = do_voxel_grid_downssampling(pcl_data, LEAF_SIZE)
    
    print("")


    # ROI 실행 코드 부분 
    # filter_axis = 'x'
    # axis_min = 0
    # axis_max = 10
    # cloud = do_passthrough(cloud, filter_axis, axis_min, axis_max)

    # filter_axis = 'y'
    # axis_min = -2
    # axis_max =2
    # cloud = do_passthrough(cloud, filter_axis, axis_min, axis_max)

    # filter_axis = 'z'
    # axis_min = -0.5
    # axis_max = 2
    filter_axis = 'x'
    axis_min = 0
    axis_max = 20
    # axis_max = 2
    # axis_max = 15
    cloud = do_passthrough(cloud, filter_axis, axis_min, axis_max)

    filter_axis = 'y'
    axis_min = -10
    axis_max = 10
    cloud = do_passthrough(cloud, filter_axis, axis_min, axis_max)

    filter_axis = 'z'
    axis_min = -0.3
    axis_max = 2

    cloud = do_passthrough(cloud, filter_axis, axis_min, axis_max)
    print("Output :", cloud)

    test = PointCloud()
    get_in = ChannelFloat32()
    get_in.name = 'intensery'
    test.points = []
    for p in cloud:
        # if p[1] > 0:
        seo = Point32()
        seo.x = p[0]
        seo.y = p[1]
        # seo.z = p[2]
        get_in.values.append(p[3])
        test.points.append(seo)
        cnt += 1
        # print(p[3])

    #print("Input :", cnt)
    test.channels.append(get_in)
    test.header.frame_id = 'world'
    test.header.stamp = rospy.Time.now()
    #test.channels = 3
    # print(test)
    pub.publish(test)


print("ON")
rospy.init_node("LiDAR", anonymous=False)
pub = rospy.Publisher("lidar_pub", PointCloud, queue_size =10)
#print(self.control_data)
#print('controller').
while not rospy.is_shutdown():
	rospy.Subscriber("/velodyne_points", PointCloud2, getMsg)
	rospy.spin()

# rospy.Subscriber('Obs', Obs, self.getObstacleMsg)
