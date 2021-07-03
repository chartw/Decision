#!/usr/bin/env python
#-*- coding:utf-8 -*-
from __future__ import division, print_function #파이썬3문법을 2에서도 쓸수있게해줌
import rospy
import math
import tf
from std_msgs.msg import Float32
from std_msgs.msg import Time
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Pose
from sensor_msgs.msg import MagneticField
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import matplotlib.pyplot as plt
import numpy as np
from numpy.random import randn
#from scipy.linalg import block_diag
#from filterpy.common import Q_discrete_white_noise
#from filterpy.stats import plot_covariance_ellipse
import pymap3d as pm
import pandas as pd
import time

import threading

# define
magnet_OFF = [0, 0, 0]
PI = 3.141592
#msg = Odometry()

# ENU base coordinates
# 송도
# base_lat = 37.448611
# base_lon = 126.654965
# base_alt = 15.4
base_lat = 37.383784
base_lon = 126.654310
base_alt = 15.4



class Localization():
    def __init__(self):
        rospy.init_node('PositionNode', anonymous=False)
        self.pub = rospy.Publisher('/pose', Odometry, queue_size = 1)

        self.msg = Odometry()
        self.e = 0
        self.n = 0
        self.u = 0

        self.dr_e = 0
        self.dr_n = 0
        self.heading = 0

    # def msg_write(self,msg):
    #     self.msg.x=float(self.e)
    #     self.msg.y=float(self.n)
    #     self.msg.heading = self.heading

    def GPS_call(self,data):
        # print("dede?")
        lon = float(data.longitude) #x
        lat = float(data.latitude) #y
        alt = float(data.altitude) #z
        self.e,self.n,self.u = pm.geodetic2enu(lat,lon,alt,base_lat,base_lon,base_alt)

        self.msg.pose.pose.position.x = self.e
        self.msg.pose.pose.position.y = self.n
      



        
        
    def IMU_call(self,data):
        #print("do?")


        quaternion = (data.orientation.x,
            data.orientation.y,
            data.orientation.z,
            data.orientation.w) 
        euler = tf.transformations.euler_from_quaternion(quaternion)

        # self.msg.twist.twist.angular.x = euler[0]*180/PI # Pitch
        # self.msg.twist.twist.angular.y = euler[1]*180/PI # Roll
        #self.msg.twist.twist.angular.z = euler[2]*180/PI # Yaw


        self.msg.twist.twist.angular.z = data.orientation.x+90
        self.msg.twist.twist.angular.z = self.msg.twist.twist.angular.z%360
        
        print(self.msg.twist.twist.angular.z)

        #칼만필터를 위해 가속도값을 메시지로 담는것
        # self.msg.pose.pose.position.z = self.msg.twist.twist.angular.z
        # if (self.msg.pose.pose.position.z > 180): 
        #     self.msg.pose.pose.position.z -= 360
   
        # orientation = tf.transformations.quaternion_from_euler(self.msg.twist.twist.angular.y*PI/180, self.msg.twist.twist.angular.x*PI/180, self.msg.pose.pose.position.z*PI/180)

        # self.msg.pose.pose.orientation.y = orientation[0]
        # self.msg.pose.pose.orientation.x = orientation[1]
        # self.msg.pose.pose.orientation.z = orientation[2]
        # self.msg.pose.pose.orientation.w = orientation[3]

    def Time_call(self,time):
        self.pub.publish(self.msg)   
        print (self.msg.twist.twist.angular.z)





loc = Localization()

rospy.Subscriber('/gps_data/fix',NavSatFix,loc.GPS_call)
rospy.Subscriber('/imu',Imu,loc.IMU_call)
rospy.Subscriber("/timer",Time, loc.Time_call)
rate = rospy.Rate(500)
rospy.spin()


