#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
import math
import pymap3d
import random
import glob
import json
import numpy as np
import geopandas as gpd
from tf.transformations import euler_from_quaternion, quaternion_from_euler

import rospy
from geometry_msgs.msg import Point
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from spline_planner import CubicSplineInterpolate
# node 작업 필요. shp 파일 알아보기. jpg 알아보기

# 얘는 csv 로 만들지도 않고, 그냥 shp 에서 node 위경도 바로 받아와서, pymap,cubicspline 다해서
# marker ,markerArray로 바로 쏴줘버림. / test.py는 Adas_map.py 의 lanelet함수만 가져와서 ,
# 164개의 lane.csv 파일 만들어줌. / draw_map.py 는 그렇게 만들어진 lane 을 matplot으로 plot 만 해줌.


# BASE_LAT, BASE_LON, BASE_ALT = 37.2389619983, 126.772982033, 29.976 
# BASE_LAT, BASE_LON, BASE_ALT = 37.237898722, 126.77174665, 29.976 # original
BASE_LAT, BASE_LON, BASE_ALT = 37.239231667, 126.773156667, 15.400 # k_city base_MINSOO




class AdasMap:
    def __init__(self):
        # global vehicle
        # ADAS Map data
        self.lanelet = {}
        self.vehicle = Odometry()

        # point-to-point distance when interpolate waypoints
        self.interval = .1
        self.get_lanelet()
        
        rospy.init_node('k_city_map',anonymous=False) # 얘 여기 있는거 맞나?? 
        print('init_node')
        
        rospy.Subscriber('/pose',Odometry,self.GPSIMU) # def(GPSIMU) 에서 /sim_gps 에서 오는 x,y,yaw담아서 markerㄱㄱ
        self.pub_vehicle = rospy.Publisher('/vehicle',Odometry,queue_size=1)
        

        # rospy.Subscriber('/fix', NavSatFix, self.rtk_gps_cb) # when rtk gps comes
        # self.pub_rtk_gps = rospy.Publisher('/rtk_gps', Marker, queue_size=1, latch=True)
        
        self.pub_vector_map = rospy.Publisher('/vector_map', MarkerArray, queue_size=1, latch=True)
        vector_map = self.get_vector_map_marker_array(self.lanelet)
        self.pub_vector_map.publish(vector_map)
    

    # def rtk_gps_cb(self, msg):  
        # self.rtk_gps = msg
        # if msg.status.status != -1: # if rtk on 
        #     x, y, z = pymap3d.geodetic2enu(msg.latitude, msg.longitude, msg.altitude, BASE_LAT, BASE_LON, BASE_ALT)
        #     # x = int((float(pt[1]) - BASE_LON)*1000000)
        #     # y = int((float(pt[0]) - BASE_LAT)*1000000)
        #     self.rtk_gps_marker.pose.position.x = x
        #     self.rtk_gps_marker.pose.position.y = y
        #     self.pub_rtk_gps.publish(self.rtk_gps_marker)  # rtk_gps_marker 는 어디서 정의 된거지 ?

    def get_distance(self, pt1, pt2):
        return math.sqrt((pt1[0]-pt2[0])**2 + (pt1[1]-pt2[1])**2)

    def get_interpolated(self, waypoints, interval):
        wx, wy = zip(*waypoints)
        csp = CubicSplineInterpolate(list(wx), list(wy))

        waypoints_out = []
        s = []
        length = 0.0
        yaw = []
        max_v = []
        k_list = []

        for ds in np.arange(0.0, csp.s[-1], interval):
            s.append(ds)
            length += ds
            x, y = csp.calc_position(ds)
            # print('11')
            yaw.append(csp.calc_yaw(ds))
            # print('22')
            k = csp.calc_curvature(ds)
            k_list.append(k)
            v = 0.0
            if k == 0.0:
                v = 150.0
            else:
                R = abs(1.0 / k)
                v = min(math.sqrt(127 * 0.25 * R), 150.0)  # 곡률에 따른 최대 속도.
            
            max_v.append(round(v,2))
            waypoints_out.append((float(x), float(y)))

        return waypoints_out, s, length, yaw, max_v, k_list

    def get_lanelet(self): # lan, lon 만 받아와 from shp.file
        data = gpd.read_file('kcity.shp')  # shp는 정밀지도 

        for i, line in enumerate(data['geometry']):
            id = str(i) # data['geometry']안에 이미 결정된 순서
            lon = line.xy[0]
            lat = line.xy[1]

            waypoints = []

            pre_pt = None

            for pt in zip(lat, lon):
                x, y, z = pymap3d.geodetic2enu(float(pt[0]), float(pt[1]), BASE_ALT, BASE_LAT, BASE_LON, BASE_ALT)
                # x = int((float(pt[1]) - BASE_LON)*1000000)
                # y = 1800 - int((float(pt[0]) - BASE_LAT)*10001.0000)
                # x, y = pt[1], pt[0]

                cur_pt = (x, y)

                if pre_pt is not None:
                    if pre_pt != cur_pt:
                        waypoints.append(cur_pt)    
                else:
                    waypoints.append(cur_pt)

                pre_pt = cur_pt

            waypoints, s, length, yaw, max_v, k = self.get_interpolated(waypoints, interval=self.interval)

            if id in self.lanelet:
                self.lanelet[id]['waypoints'] = waypoints
                self.lanelet[id]['s'] = s
                self.lanelet[id]['length'] = length
                self.lanelet[id]['yaw'] = yaw
                self.lanelet[id]['max_v'] = max_v
                self.lanelet[id]['k'] = k
            else:
                self.lanelet[id] = {'waypoints':waypoints, 's':s, 'length':length, 'yaw':yaw, 'max_v':max_v, 'k':k}

    # def get_lane_marker(self, name_space, id): # 이거 안쓰고 있음
    #     marker = Marker()
    #     marker.type = Marker.LINE_STRIP
    #     marker.action = Marker.ADD
    #     marker.header.frame_id = 'world'
    #     marker.ns = name_space
    #     marker.id = int(id)
    #     marker.lifetime = rospy.Duration(0)
    #     marker.scale.x = 0.3
    #     marker.color.r = 1.0
    #     marker.color.g = 1.0
    #     marker.color.b = 1.0
    #     marker.color.a = 1.0
    #     marker.pose.orientation.x = 0.0
    #     marker.pose.orientation.y = 0.0
    #     marker.pose.orientation.z = 0.0
    #     marker.pose.orientation.w = 1.0
    #     return marker

    def get_waypoints_marker(self, name_space, id, color):
        marker = Marker()
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.header.frame_id = 'world'
        marker.ns = name_space
        marker.id = int(id)
        marker.lifetime = rospy.Duration(0)
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = 1.0
        return marker

    def get_text_marker(self, id, color):
        marker = Marker()
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.header.frame_id = 'world'
        marker.ns = 'lanelet_id'
        marker.id = int(id)
        marker.lifetime = rospy.Duration(0)
        marker.scale.z = 2.5 # rviz 글씨 크기 조절하는애 얘임.
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = 1.0
        return marker

    def get_vector_map_marker_array(self, lanelet):
        vector_map_marker = MarkerArray() #(pt[1]) - BASE_LON)*1000000)
                # y = 1800 - int
        
        # marker ARRAY[] 에 color있는 waypoints marker 도 append.
        for id in lanelet:
            color = (random.random(), random.random(), random.random())
            marker = self.get_waypoints_marker('waypoints', id, color)
            for x, y in lanelet[id]['waypoints']:
                marker.points.append(Point(x=x, y=y, z=0.0))
            vector_map_marker.markers.append(marker)

            # text marker. mid point of lanelet 지점에 써주도록 append.
            marker = self.get_text_marker(id, color) 
            mid = len(lanelet[id]['waypoints']) // 2
            t_idx = 0
            while True:
                t_idx = mid + random.randrange(-15, 15)
                if t_idx >= 0 and t_idx < len(lanelet[id]['waypoints']):
                    break

            mid_x, mid_y = lanelet[id]['waypoints'][t_idx]
            marker.text = id
            marker.pose.position.x = mid_x
            marker.pose.position.y = mid_y
            marker.pose.position.z = 0.0
            vector_map_marker.markers.append(marker)
        return vector_map_marker


    ###--------- GPSIMU 함수로 lon,lat > x,y[m]로 해서 다시 Odometry publish. ----------------------

    def GPSIMU(self,Odo): #converted lat->x, lon->y for display now car's position
        global BASE_LAT, BASE_LON, BASE_ALT 
        # global vehicle

        lon= Odo.pose.pose.position.x
        lat= Odo.pose.pose.position.y
        alt = 15.4
        
        x, y, u = pymap3d.geodetic2enu(lat, lon, alt, BASE_LAT, BASE_LON, BASE_ALT)

        # Publish Odometry msg of vehicle in ROS
        self.vehicle.header.stamp = rospy.Time.now()
        self.vehicle.header.frame_id ='world'
        self.vehicle.child_frame_id='world'
        self.vehicle.pose.pose.position.x = x
        self.vehicle.pose.pose.position.y = y
        yaw_rad=math.radians(Odo.twist.twist.angular.z)
        q = quaternion_from_euler(0,0,yaw_rad)
        # print(q)
        self.vehicle.pose.pose.orientation.x=q[0]
        self.vehicle.pose.pose.orientation.y=q[1]
        self.vehicle.pose.pose.orientation.z=q[2]
        self.vehicle.pose.pose.orientation.w=q[3]
        
        # self.vehicle.twist.twist.angular.z = math.radians(Odo.twist.twist.angular.z)
        print('vehicle[m]:{},{} ,yaw:{}'.format(self.vehicle.pose.pose.position.x,self.vehicle.pose.pose.position.y,Odo.twist.twist.angular.z)) # 여까지 vehicle msg에 담아만 둠. 
        self.pub_vehicle.publish(self.vehicle) # vehicle=Odometry

if __name__=='__main__':
    AdasMap()
    rospy.spin()
