#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys,os
import rospy
import rospkg
import numpy as np
from nav_msgs.msg import Path,Odometry
from std_msgs.msg import Float64,Int16,Float32MultiArray
from geometry_msgs.msg import PoseStamped,Point
from morai_msgs.msg import ERP42Info,ObjectInfo,CtrlCmd,GetTrafficLightStatus,SetTrafficLight
from lib.utils import pathReader, findLocalPath,purePursuit,cruiseControl,vaildObject,pidController,velocityPlanning,latticePlanner
import tf
from math import cos,sin,sqrt,pow,atan2,pi

class erp_planner():
    def __init__(self):
        rospy.init_node('erp42_total', anonymous=True)

        arg = rospy.myargv(argv=sys.argv)
        self.path_name=arg[1]
        self.traffic_control=arg[2]
        


        #publisher
        global_path_pub= rospy.Publisher('/global_path',Path, queue_size=1)
        local_path_pub= rospy.Publisher('/local_path',Path, queue_size=1)
        ctrl_pub = rospy.Publisher('/ctrl_cmd',CtrlCmd, queue_size=1)
        traffic_pub = rospy.Publisher("/SetTrafficLight",SetTrafficLight,queue_size=1)

        ########################  lattice  ########################
        for i in range(1,8):            
            globals()['lattice_path_{}_pub'.format(i)]=rospy.Publisher('lattice_path_{}'.format(i),Path,queue_size=1)  

        ########################  lattice  ########################

        ctrl_msg= CtrlCmd()
        
        #subscriber
        rospy.Subscriber("/Ego_topic", ERP42Info, self.statusCB)
        rospy.Subscriber("/Object_topic", ObjectInfo, self.objectInfoCB)
        rospy.Subscriber("/GetTrafficLightStatus", GetTrafficLightStatus, self.getTL_callback)

        #def
        self.is_status=False
        self.is_obj=False
        self.is_traffic=False
        self.traffic_info = [[58.50, 1180.41 ,'C119BS010001'], ##신호등 정보(global_x, global_y, index)
                             [85.61, 1227.88 ,'C119BS010021'],
                             [136.58,1351.98 ,'C119BS010026'],
                             [141.02,1458.27 ,'C119BS010028'],
                             [139.39,1596.44 ,'C119BS010033'],
                             [48.71, 1208.02 ,'C119BS010005'],
                             [95.58, 1181.56 ,'C119BS010047'],
                             [104.46,1161.46 ,'C119BS010046'],
                             [85.29, 1191.77 ,'C119BS010007'],
                             [106.32,1237.04 ,'C119BS010022'],
                             [75.34, 1250.43 ,'C119BS010024'],
                             [73.62, 1218.01 ,'C119BS010012'],
                             [116.37,1190.65 ,'C119BS010040'],
                             [153.98,1371.48 ,'C119BS010073'],
                             [129.84,1385.08 ,'C119BS010039'],
                             [116.28,1367.77 ,'C119BS010074'],
                             [75.08, 1473.34 ,'C119BS010075'],
                             [67.10, 1506.66 ,'C119BS010076'],
                             [114.81,1485.81 ,'C119BS010079'],
                             [159.11,1496.63 ,'C119BS010060'],
                             [122.24,1608.26 ,'C119BS010072'],
                             [132.70,1624.78 ,'C119BS010034']]
      

        #class
        path_reader=pathReader('erp_ros')
        pure_pursuit=purePursuit()
        self.cc=cruiseControl(0.5,1)
        self.vo=vaildObject(self.traffic_info)
        pid=pidController()       

        
        #read path
        self.global_path=path_reader.read_txt(self.path_name+".txt")
        
        self.object_info_msg=ObjectInfo()
        self.status_msg=ERP42Info()

        vel_planner=velocityPlanning(200/3.6,1.5)
        vel_profile=vel_planner.curveBasedVelocity(self.global_path,100)

        #time var
        count=0
        rate = rospy.Rate(30) # 30hz

        lattice_current_lane=3

        while not rospy.is_shutdown():

            if self.is_status==True  and self.is_obj==True:
                local_path,self.current_waypoint=findLocalPath(self.global_path,self.status_msg)
                
                self.vo.get_object(self.object_info_msg.num_of_objects,self.object_info_msg.object_type,self.object_info_msg.pose_x,self.object_info_msg.pose_y,self.object_info_msg.velocity)
                global_obj,local_obj=self.vo.calc_vaild_obj([self.status_msg.position_x,self.status_msg.position_y,(self.status_msg.yaw+90)/180*pi])

                ########################  lattice  ########################
                vehicle_status=[self.status_msg.position_x,self.status_msg.position_y,(self.status_msg.yaw+90)/180*pi,self.status_msg.velocity/3.6]
                lattice_path,selected_lane=latticePlanner(local_path,global_obj,vehicle_status,lattice_current_lane)
                lattice_current_lane=selected_lane
                                
                if selected_lane != -1: 
                    local_path=lattice_path[selected_lane]                
                
                if len(lattice_path)==7:                    
                    for i in range(1,8):
                        globals()['lattice_path_{}_pub'.format(i)].publish(lattice_path[i-1])
                ########################  lattice  ########################

                if self.is_traffic == True:
                    if self.traffic_control == "True":
                        self.tl_msg.trafficLightStatus=16
                        ###################### traffic_control ######################
                        self.set_traffic_data= SetTrafficLight()
                        self.set_traffic_data.trafficLightIndex = self.tl_msg.trafficLightIndex
                        self.set_traffic_data.trafficLightStatus = 16 ##set greenlight 
                        traffic_pub.publish(self.set_traffic_data)

                    self.cc.checkObject(local_path,global_obj,local_obj,[self.tl_msg.trafficLightIndex,self.tl_msg.trafficLightStatus])

                else :
                    self.cc.checkObject(local_path,global_obj,local_obj)

                pure_pursuit.getPath(local_path)
                pure_pursuit.getEgoStatus(self.status_msg)

                ctrl_msg.steering=pure_pursuit.steering_angle()

                cc_vel = self.cc.acc(local_obj,self.status_msg.velocity,vel_profile[self.current_waypoint],self.status_msg)
                
                target_velocity = cc_vel
                ctrl_msg.velocity=cc_vel
                

                control_input=pid.pid(target_velocity,self.status_msg.velocity) ## 속도 제어를 위한 PID 적용 (target Velocity, Status Velocity)
                
                if control_input > 0 :
                    ctrl_msg.accel= control_input
                    ctrl_msg.brake= 0

                else :
                    ctrl_msg.accel= 0
                    ctrl_msg.brake= -control_input

                if self.status_msg.velocity < 3.0  and target_velocity<=0.0:
                    ctrl_msg.accel=0
                    ctrl_msg.brake=1
                

                local_path_pub.publish(local_path)
                ctrl_pub.publish(ctrl_msg)
            
            
            if count/300==1 :
                global_path_pub.publish(self.global_path)
                count=0
            count+=1
            self.steering_angle=ctrl_msg.steering
            self.print_info()
            rate.sleep()


    def print_info(self):

        os.system('clear')
        print('--------------------status-------------------------')
        print('position :{0} ,{1}, {2}'.format(self.status_msg.position_x,self.status_msg.position_y,self.status_msg.position_z))
        print('velocity :{} km/h'.format(self.status_msg.velocity))
        print('heading :{} deg'.format(self.status_msg.yaw-90))

        print('--------------------object-------------------------')
        print('object num :{}'.format(self.object_info_msg.num_of_objects))
        for i in range(0,self.object_info_msg.num_of_objects) :
            print('{0} : type = {1}, x = {2}, y = {3}, z = {4} '.format(i,self.object_info_msg.object_type[i],self.object_info_msg.pose_x[i],self.object_info_msg.pose_y[i],self.object_info_msg.pose_z[i]))

        print('--------------------controller-------------------------')
        print('target steering_angle :{} deg'.format(self.steering_angle))

        print('--------------------localization-------------------------')
        print('all waypoint size: {} '.format(len(self.global_path.poses)))
        print('current waypoint : {} '.format(self.current_waypoint))

        if self.is_traffic==True:
            print('--------------------trafficLight-------------------------')        
            print('traffic mode : {}'.format(self.tl_msg.isAutoMode))
            print('traffic index : {}'.format(self.tl_msg.trafficLightIndex))
            print('traffic type : {}'.format(self.tl_msg.trafficLightType))
            print('traffic status : {}'.format(self.tl_msg.trafficLightStatus))


    def statusCB(self,data):
        self.status_msg=ERP42Info()
        self.status_msg=data
        br = tf.TransformBroadcaster()
        br.sendTransform((self.status_msg.position_x, self.status_msg.position_y, self.status_msg.position_z),
                        tf.transformations.quaternion_from_euler(0, 0, (self.status_msg.yaw+90)/180*pi),
                        rospy.Time.now(),
                        "gps",
                        "map")
        self.is_status=True                 

    def objectInfoCB(self,data):
        self.object_info_msg=ObjectInfo()
        self.object_info_msg=data
        self.is_obj=True

    def getTL_callback(self,msg):
        self.is_traffic=True
        self.tl_msg=GetTrafficLightStatus()
        self.tl_msg=msg
        

    
if __name__ == '__main__':
    try:
        kcity_pathtracking=erp_planner()
    except rospy.ROSInterruptException:
        pass
