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
from lib.planner_utils.mission_plan import MissonPlan
from lib.planner_utils.traffic_state import traffic_light_3,traffic_light_4

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

        planning_info_pub = rospy.Publisher("/planner", Planning_Info, queue_size=1)

        # subscriber 정의
        self.planning_msg = Planning_Info()
        self.obstacle_msg = Obstacles()
        # self.object_msg = BoundingBoxes()
        self.surface_msg = String()
        self.serial_msg = Serial_Info()
        self.parking_msg=Int16()
        
        # LiDAR      
        # rospy.Subscriber("/obstacles", Obstacles, self.obstacleCallback)
        # rospy.Subscriber("/parking",Int16, self.parkingCallback)

        # Localization        
        rospy.Subscriber("/pose", Odometry, self.localCallback)
        
        # Vision - Object
        # def objectCallback(self, msg): self.object_msg = msg
        # rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, self.objectCallback)   
        
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
        self.mission_goal=Point32()

        # gpp 변수 선언
        global_path_maker = GPP(self)
        # local_point_maker = LPP(self)
        # misson_planner = MissonPlan(self)

        # 신호등 state time count 변수
        self.global_time = time.time()    
        # 신호등 state 사용예제
        self.traffic3_1 = traffic_light_3() # 3구 신호등1 선언

        rate = rospy.Rate(50)  # 100hz

        while not rospy.is_shutdown():
    
            if self.is_local:
                # gpp가 필요하고, 위치 정보가 들어와 있을 때 gpp 실행
                if self.gpp_requested:
                    self.global_path = global_path_maker.path_plan()
                    self.planning_msg.path_x = self.global_path.x
                    self.planning_msg.path_y = self.global_path.y
                    self.planning_msg.path_heading = self.global_path.heading
                    self.planning_msg.path_k = self.global_path.k
                    self.planning_msg.mode="general"
                    self.gpp_requested = False
                    
                # else:
                #     self.planning_msg.mode=misson_planner.decision(self)

                #     if self.planning_msg.mode=="avoidance":
                #         if len(self.obstacle_msg.segments) !=0:
                #             self.planning_msg.point=local_point_maker.point_plan()
                #             point=self.planning_msg.point
                #             theta=self.local.heading*pi/180
                #             self.mission_goal.x=point.x*cos(theta)+point.y*-sin(theta) + self.local.x
                #             self.mission_goal.y=point.x*sin(theta)+point.y*cos(theta) + self.local.y

                #     # elif self.planning_msg.mode=="parking-start":
                #         # self.planning_msg.path=
                

                self.planning_msg.local=self.local
                planning_info_pub.publish(self.planning_msg)

                if not self.gpp_requested:
                    self.planning_msg.path_x = []
                    self.planning_msg.path_y = []
                    self.planning_msg.path_heading = []
                    self.planning_msg.path_k = []
                    
                rate.sleep()


    # Callback Function
    def obstacleCallback(self, msg): 
        self.obstacle_msg = msg  
    
    def localCallback(self, msg):
        # print(self.local)
        self.local.x = msg.pose.pose.position.x
        self.local.y = msg.pose.pose.position.y
        self.local.heading = msg.twist.twist.angular.z
        self.is_local = True

    def surfaceCallback(self, msg): 
        self.surface_msg = msg
        
    def serialCallback(self, msg):
        self.serial_msg = msg


    def change_traffic_state3(self):
        if (self.traffic_state=='red' and self.traffic_global_time)
         


        
         
if __name__ == "__main__":
    planner = Planner()
    planner.run()
