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
from lib.planner_utils.mapping import Mapping


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
        self.local_path_pub = rospy.Publisher("/local_path2", PointCloud, queue_size=1)
        self.obs_pub = rospy.Publisher("/obs_pub2", PointCloud, queue_size=1)
        self.pose_pub = rospy.Publisher("/pose_pub", PointCloud, queue_size=1)
        self.global_path_pub = rospy.Publisher("/global_path", PointCloud, queue_size=1)

        # subscriber 정의
        self.planning_msg = Planning_Info()
        self.obstacle_msg = Obstacles()
        self.object_msg = String()
        # self.object_msg = BoundingBoxes()
        self.object_msg = String()  # temporary setting for development
        self.surface_msg = String()
        self.serial_msg = Serial_Info()
        self.parking_msg = Int16()

        # LiDAR
        rospy.Subscriber("/obstacles", Obstacles, self.obstacleCallback)
        # rospy.Subscriber("/parking",Int16, self.parkingCallback)

        # Localization
        rospy.Subscriber("/pose", Odometry, self.localCallback)

        # Vision - Object
        # def objectCallback(self, msg): self.object_msg = msg
        rospy.Subscriber("/darknet_ros/bounding_boxes", String, self.objectCallback)

        rospy.Subscriber("/serial", Serial_Info, self.serialCallback)

        # Vision - Surface
        rospy.Subscriber("/surface", String, self.surfaceCallback)

        # 상태 flag
        self.is_local = False
        self.is_obstacle = False
        self.is_object = False
        self.gpp_requested = True
        self.is_global_path_pub = False
        self.mission_ing = False
        self.is_avoidance_ing = False

        # data 변수 선언
        self.global_path = Path()
        self.local_path = Path()
        self.local = Local()
        # self.objects = BoundingBoxes()
        self.is_person = False
        self.mission_goal = Point32()

        self.veh_index = 0
        self.target_index = 0

        # gpp 변수 선언
        self.global_path_maker = GPP(self)
        self.local_path_maker = LPP(self)
        self.misson_planner = MissionPlan(self)
        self.map_maker = Mapping(self)

        self.past_state = 0

        self.vis_local_path = PointCloud()
        self.vis_local_path.header.frame_id = "world"

        self.vis_global_path = PointCloud()
        self.vis_global_path.header.frame_id = "world"

        self.obs = PointCloud()
        self.obs.header.frame_id = "world"

        self.pose = PointCloud()
        self.pose.header.frame_id = "world"

    def run(self):
        rate = rospy.Rate(50)  # 100hz

        while not rospy.is_shutdown():
            if self.is_local:
                # GPP
                if self.gpp_requested:

                    self.global_path = self.global_path_maker.path_plan()
                    self.planning_msg.mode = "general"
                    self.gpp_requested = False

                # Localization Information
                self.planning_msg.local = self.local

                # Mission Decision
                self.planning_msg.state = self.misson_planner.state_check(self)

                if not self.mission_ing:
                    self.planning_msg.mode, self.mission_ing = self.misson_planner.decision(self)
                else:
                    self.mission_ing = self.misson_planner.end_check(self)  # return True/False

                if self.planning_msg.mode == "general":
                    self.planning_msg.path = self.global_path
                    self.target_index, self.planning_msg.point = self.global_path_maker.point_plan(self, 4)

                elif self.planning_msg.mode == "avoidance":
                    if self.is_avoidance_ing == False:
                        self.is_avoidance_ing = True
                        self.local_path_maker.start(self)

                    points=self.map_maker.showObstacleMap().points
                    self.local_path = self.local_path_maker.path_plan(points)

                    if self.local_path.x:
                        self.planning_msg.path = self.local_path
                        self.planning_msg.point = self.local_path_maker.point_plan(self, 2)

                self.vis_local_path.points = []
                for i in range(len(self.local_path.x)):
                    self.vis_local_path.points.append(Point32(self.local_path.x[i], self.local_path.y[i], 0))
                self.vis_local_path.header.stamp = rospy.Time.now()
                self.local_path_pub.publish(self.vis_local_path)

                if len(self.vis_global_path.points) == 0:
                    for i in range(len(self.global_path.x)):
                        self.vis_global_path.points.append(Point32(self.global_path.x[i], self.global_path.y[i], 0))
                        self.vis_global_path.header.stamp = rospy.Time.now()
                self.global_path_pub.publish(self.vis_global_path)

                # self.localpoint.points.append(self.planning_msg.point)
                # self.localpoint.header.stamp=rospy.Time.now()
                # self.point_pub.publish(self.localpoint)

                self.obs.points= self.map_maker.showObstacleMap().points
                self.obs.header.stamp = rospy.Time.now()
                self.obs_pub.publish(self.obs)

                self.pose.points = []
                self.pose.points.append(Point32(self.local.x, self.local.y, 0))
                self.pose.header.stamp = rospy.Time.now()
                self.pose_pub.publish(self.pose)

                self.planning_info_pub.publish(self.planning_msg)
                self.past_state = self.planning_msg.state
                rate.sleep()

    # Callback Functions
    def obstacleCallback(self, msg):
        self.obstacle_msg.segments = msg.segments
        self.obstacle_msg.circles = msg.circles
        self.obstacle_msg.circle_number = msg.circle_number

        self.map_maker.mapping(self,msg.circles)

    def localCallback(self, msg):
        self.local.x = msg.pose.pose.position.x
        self.local.y = msg.pose.pose.position.y
        self.local.heading = msg.twist.twist.angular.z
        self.is_local = True

    def surfaceCallback(self, msg):
        self.surface_msg = msg

    def serialCallback(self, msg):
        self.serial_msg = msg

    def objectCallback(self, msg):
        self.object_msg.data = msg.data


if __name__ == "__main__":
    planner = Planner()
    planner.run()
