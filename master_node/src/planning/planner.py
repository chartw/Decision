#!/usr/bin/env python3
# -*- coding:utf-8 -*-
import rospy

import numpy as np
from math import radians, degrees, sin, cos, hypot, atan2, pi
import sys
import time
from master_node.msg import Obstacles, PangPang, Planning_Info, Path, Local, Serial_Info
from nav_msgs.msg import Odometry
# from darknet_ros_msgs.msg import BoundingBoxes
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from std_msgs.msg import Float32, Time, String, Int16

# from lane_detection.msg import lane
from lib.planner_utils.global_path_plan import GPP
from lib.planner_utils.local_point_plan import LPP
from lib.planner_utils.mission_plan import MissionPlan

class Planner:
    def __init__(self):
        rospy.init_node("Planner", anonymous=False)

        # 맵 이름이랑, 도착노드 system argument로 받아서 실행하자~
        # ex) python3 planner.py songdo 38
        # 후에는 roslaunch 파일로 바꾸면서 parameter 가져오도록 변경
        arg = rospy.myargv(argv=sys.argv)
        # arg[0] == planner.py
        self.map = str(arg[1])
        self.goal_node = str(arg[2])

        """
        publish 정의
        Planning_Info
        {
            String mode
            Local local
            Path path
            Point32 point
        }
        """

        self.planning_info_pub = rospy.Publisher("/planner", Planning_Info, queue_size=1)
        point_pub=rospy.Publisher("/local_point",PointCloud,queue_size=1)

        # subscriber 정의
        self.planning_msg = Planning_Info()
        self.obstacle_msg = Obstacles()
        self.object_msg=String()
        # self.object_msg = BoundingBoxes()
        self.object_msg = String() #temporary setting for development
        self.surface_msg = String()
        self.serial_msg = Serial_Info()
        self.parking_msg=Int16()
        
        # LiDAR      
        rospy.Subscriber("/obstacles", Obstacles, self.obstacleCallback)
        # rospy.Subscriber("/parking",Int16, self.parkingCallback)

        # Localization        
        rospy.Subscriber("/pose", Odometry, self.localCallback)
        
        # Vision - Object
        # def objectCallback(self, msg): self.object_msg = msg
        rospy.Subscriber("/darknet_ros/bounding_boxes", String, self.objectCallback)   
        
        rospy.Subscriber('/serial', Serial_Info, self.serialCallback)
        
        # Vision - Surface
        rospy.Subscriber("/surface", String, self.surfaceCallback)


        # 상태 flag
        self.is_local = False
        self.is_obstacle = False
        self.is_object = False
        self.gpp_requested = True
        self.is_global_path_pub = False
        self.mission_ing = False

        # data 변수 선언
        self.global_path = Path()
        self.local = Local()
        # self.objects = BoundingBoxes()
        self.is_person = False
        self.mission_goal = Point32()

        # gpp 변수 선언
        global_path_maker = GPP(self)
        local_point_maker = LPP()
        misson_planner = MissonPlan(self)

        self.localpoint=PointCloud()
        self.localpoint.header.frame_id='world'

        

    def run(self):
        rate = rospy.Rate(50)  # 100hz

        while not rospy.is_shutdown() and self.is_local:
            # GPP
            if self.gpp_requested:
                self.global_path = global_path_maker.path_plan()
                self.planning_msg.path_x = self.global_path.x
                self.planning_msg.path_y = self.global_path.y
                self.planning_msg.path_heading = self.global_path.heading
                self.planning_msg.path_k = self.global_path.k
                self.planning_msg.mode="general"
                self.gpp_requested = False
            else: #gpp not requested
                self.planning_msg.path_x = []
                self.planning_msg.path_y = []
                self.planning_msg.path_heading = []
                self.planning_msg.path_k = []
            
             
            #Localization Information
            self.planning_msg.local=self.local

            # Mission Decision
            if not self.mission_ing:
                self.planning_msg.mode=MissionPlan.decision(self)                      
            else:
                self.mission_ing=MissionPlan.end_check() #return True/False
                
                ##
            if self.planning_msg.mode=="avoidance":
                if self.obstacle_msg.segments:
                    self.planning_msg.point=local_point_maker.point_plan(self.obstacle_msg.segments)
                    point=self.planning_msg.point
                    theta=self.local.heading*pi/180
                    self.mission_goal.x=point.x*cos(theta)+point.y*-sin(theta) + self.local.x
                    self.mission_goal.y=point.x*sin(theta)+point.y*cos(theta) + self.local.y

                #######
                self.localpoint.points.append(self.planning_msg.point)
                self.localpoint.header.stamp=rospy.Time.now()
                point_pub.publish(self.localpoint)
                #######
                
            self.planning_info_pub.publish(self.planning_msg)
            rate.sleep()



    # Callback Functions
    def obstacleCallback(self, msg): 
        self.obstacle_msg.segments = msg.segments
        self.obstacle_msg.circles = msg.circles
        self.obstacle_msg.circle_number = msg.circle_number
    
    def localCallback(self, msg):
        self.local.x = msg.pose.pose.position.x
        self.local.y = msg.pose.pose.position.y
        self.local.heading = msg.twist.twist.angular.z
        self.is_local = True

    def surfaceCallback(self, msg): 
        self.surface_msg = msg
        
    def serialCallback(self, msg):
        self.serial_msg = msg

    def objectCallback(self,msg):
        self.object_msg.data=msg.data


if __name__ == "__main__":
    planner = Planner()
