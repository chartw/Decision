#!/usr/bin/env python3
# -*- coding:utf-8 -*-
import rospy

import numpy as np
from math import radians, degrees, sin, cos, hypot, atan2, pi
import sys
import time
from master_node.msg import Obstacles, PangPang, Planning_Info, Path, Local, Serial_Info
from nav_msgs.msg import Odometry

from darknet_ros_msgs.msg import BoundingBoxes
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from std_msgs.msg import Float32, Time, String, Int16, Int32

# from lane_detection.msg import lane
from lib.planner_utils.global_path_plan import GPP
from lib.planner_utils.local_point_plan import LPP
from lib.planner_utils.mission_plan import MissionPlan
from lib.planner_utils.mapping import Mapping
from lib.planner_utils.parking_path_plan import ParkingPlan
from lib.planner_utils.stopline_check import StopLine
from lib.planner_utils.trafficLight import trafficLight
from lib.planner_utils.delivery import deliveryClass


class Planner:
    def __init__(self):
        rospy.init_node("Planner", anonymous=False)

        # 맵 이름이랑, 도착노드 system argument로 받아서 실행하자~
        # ex) python3 planner.py songdo 38
        # 후에는 roslaunch 파일로 바꾸면서 parameter 가져오도록 변경
        arg = rospy.myargv(argv=sys.argv)
        # arg[0] == planner.py
        self.map = str(arg[1])
        if self.map == "songdo" or self.map == "kcity":
            self.goal_node = str(arg[2])

        else:
            if len(arg) >= 3:
                self.veh_index = int(arg[2])
            else:
                self.veh_index = 0
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

        # 상태 flag
        self.is_local = False
        self.is_obstacle = False
        self.is_object = False
        self.gpp_requested = True
        self.is_global_path_pub = False
        self.mission_ing = False
        self.is_avoidance_ing = False

        # subscriber 정의
        self.planning_msg = Planning_Info()
        self.obstacle_msg = Obstacles()
        self.object_msg = BoundingBoxes()
        self.surface_msg = String()
        self.serial_msg = Serial_Info()
        self.parking_msg = Int32()

        # data 변수 선언
        self.global_path = Path()
        self.local_path = Path()
        self.parking_path = Path()
        self.parking_backpath = Path()
        self.local = Local()
        # self.objects = BoundingBoxes()
        self.is_person = False
        self.mission_goal = Point32()
        self.target_map = {}
        self.sign_map = {}
        self.past_dist = 0
        self.past_min_dist = 0
        self.start_time = time.time()
        self.dynamic_flag = False
        self.check_veh_index_first = True

        # self.veh_index = 0
        self.target_index = 0
        self.stop_index = 0

        # gpp 변수 선언
        self.global_path_maker = GPP(self)
        self.local_path_maker = LPP(self)
        self.parking_planner = ParkingPlan(self)

        self.misson_planner = MissionPlan(self)
        self.map_maker = Mapping(self)
        self.stop_line_checker = StopLine()
        self.traffic_light = trafficLight()
        self.delivery_decision = deliveryClass()

        self.parking_planner

        self.vis_parking_path = PointCloud()
        self.vis_parking_path.header.frame_id = "world"

        self.vis_local_path = PointCloud()
        self.vis_local_path.header.frame_id = "world"

        self.vis_global_path = PointCloud()
        self.vis_global_path.header.frame_id = "world"

        self.map = PointCloud()
        self.map.header.frame_id = "world"

        self.obs = PointCloud()
        self.obs.header.frame_id = "world"

        self.pose = PointCloud()
        self.pose.header.frame_id = "world"

        self.target = PointCloud()
        self.target.header.frame_id = "world"

        self.pmode = ""
        self.is_parking = False
        self.parking_target = 1
        self.veh_index = 0
        self.parking_target_index = 0
        self.target_index = 0
        self.target_b = -1

        self.is_delivery = False
        self.dmode = ""

        self.count = 0
        self.booleanFalse = False

        self.planning_info_pub = rospy.Publisher("/planner", Planning_Info, queue_size=1)
        self.local_path_pub = rospy.Publisher("/local_path", PointCloud, queue_size=1)
        self.parking_path_pub = rospy.Publisher("/parking_path", PointCloud, queue_size=1)
        self.map_pub = rospy.Publisher("/map_pub", PointCloud, queue_size=1)
        self.obs_pub = rospy.Publisher("/obs_pub", PointCloud, queue_size=1)
        self.pose_pub = rospy.Publisher("/pose_pub", PointCloud, queue_size=1)
        self.global_path_pub = rospy.Publisher("/global_path", PointCloud, queue_size=1)
        self.target_pub = rospy.Publisher("/target", PointCloud, queue_size=1)

        # LiDAR
        rospy.Subscriber("/obstacles", Obstacles, self.obstacleCallback)
        rospy.Subscriber("/Parking_num", Int32, self.parkingCallback)

        # Localization
        rospy.Subscriber("/pose", Odometry, self.localCallback)

        # Vision - Object
        # def objectCallback(self, msg): self.object_msg = msg
        rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, self.objectCallback)
        # rospy.Subscribeer("/")

        rospy.Subscriber("/serial", Serial_Info, self.serialCallback)

        # Vision - Surface
        rospy.Subscriber("/surface", String, self.surfaceCallback)

    def check_dist(self):
        obs_dist = -1
        id = -1
        for i, obstacle in self.map_maker.obs_map.items():
            if obstacle.dist < 1.5:
                dist = (obstacle.index - self.veh_index) / 10
                if dist < 0:
                    continue
                if obs_dist > dist or obs_dist == -1:
                    id = i
                    obs_dist = dist

        if id != -1:
            min_dist = self.map_maker.obs_map[id].dist
        else:
            min_dist = -1

        if abs(self.past_dist - obs_dist) > 0.1 or abs(self.past_min_dist - min_dist) > 0.01:
            self.start_time = time.time()

        self.past_dist = obs_dist
        self.past_min_dist = min_dist

        return obs_dist

    def get_veh_index(self):
        min_dis = -1
        min_idx = 0
        if self.check_veh_index_first:  # cur_idx 잡는데, 배달미션이나 cross 되는부분은 ,
            for i in range(len(self.global_path.x)):
                dis = hypot(self.global_path.x[i] - self.local.x, self.global_path.y[i] - self.local.y)
                if min_dis > dis or min_dis == -1:  # 여기에 등호가 붙으면, 뒷부분 index 잡고, 안붙으면 앞쪽 index
                    min_dis = dis
                    min_idx = i
            self.check_veh_index_first = False
        else:
            for i in range(max(self.veh_index - 50, 0), self.veh_index + 50):
                dis = hypot(self.global_path.x[i] - self.local.x, self.global_path.y[i] - self.local.y)
                if min_dis > dis or min_dis == -1:
                    min_dis = dis
                    min_idx = i

        return min_idx

    def run(self):
        rate = rospy.Rate(50)  # 100hz

        while not rospy.is_shutdown():
            # print("current point is:", self.local.x, self.local.y)
            # print(self.veh_index)
            if self.is_local:
                # print(self.planning_msg.mode)

                # GPP
                # print(self.planning_msg.mode)
                # print(self.planning_msg.dist)
                if self.gpp_requested:
                    self.global_path = self.global_path_maker.path_plan()
                    self.planning_msg.mode = "general"
                    self.gpp_requested = False

                # endcheck = False
                # Localization Information
                self.planning_msg.local = self.local
                self.veh_index = self.get_veh_index()
                # self.stop_index = self.stop_line_checker.stop_idx_check(planner)
                # print(self.stop_index)

                if not self.is_parking:
                    self.planning_msg.dist = self.check_dist()
                    # self.planning_msg.mode = self.misson_planner.decision(self)

                if self.planning_msg.mode == "general" or self.planning_msg.mode == "kid" or self.planning_msg.mode == "bump":
                    self.planning_msg.path = self.global_path

                if self.planning_msg.mode == "small" or self.planning_msg.mode == "big":

                    self.target_map = self.map_maker.make_target_map(self)
                    self.local_path = self.local_path_maker.path_plan(self.target_map)

                    if self.local_path.x:
                        self.planning_msg.path = self.local_path
                #     if self.local_path.x:
                #         self.planning_msg.path = self.local_path
                #         self.planning_msg.point = self.local_path_maker.point_plan(self, 2)
                elif self.planning_msg.mode == "parking":
                    self.is_parking = True

                elif self.planning_msg.mode == "delivery1":
                    self.is_delivery = True

                elif self.planning_msg.mode == "crossroad":
                    self.planning_msg.mode = "general"

                    # self.planning_msg.dist=(self.stop_index-self.veh_index)/10
                    # signal = self.traffic_light.run(self.object_msg.bounding_boxes) # string
                    # print(signal)
                    # if self.global_path.mission[self.stop_index] in signal:
                    #     self.planning_msg.mode="general"
                    # else:
                    #     self.planning_msg.mode="normal_stop"

                #####Parking
                if self.is_parking is True:
                    self.pmode = self.parking_planner.parking_state_decision(self)

                    print("Current Mission: ", self.planning_msg.mode)

                    # if self.pmode == "parking-base1":
                    #     self.parking_path = self.parking_planner.make_parking_path(1)

                    if self.pmode == "parking_ready":
                        self.parking_path = self.parking_planner.make_parking_path(self.parking_target)

                        for i in range(len(self.parking_path.x)):
                            self.parking_backpath.x.insert(0, self.parking_path.x[i])
                            self.parking_backpath.y.insert(0, self.parking_path.y[i])
                        print("==========parking path created")
                        print(self.parking_path)
                        self.vis_parking_path.points = []
                        for i in range(len(self.parking_path.x)):
                            self.vis_parking_path.points.append(Point32(self.parking_backpath.x[i], self.parking_backpath.y[i], 0))
                        self.vis_parking_path.header.stamp = rospy.Time.now()
                        self.parking_path_pub.publish(self.vis_parking_path)

                    elif self.pmode == "parking_start":
                        self.parking_target_index, self.planning_msg.point = self.parking_planner.point_plan(self.parking_path, 3)

                    elif self.pmode == "parking_backward":
                        self.parking_target_index, self.planning_msg.point = self.parking_planner.point_plan(self.parking_backpath, 3)

                    elif self.pmode == "parking_end":
                        self.pmode = "general"
                        self.mission_ing = False
                        self.is_parking = False

                    self.planning_msg.mode = self.pmode

                #####################SHSHSSHSHSHSHSHSHSHSSHSSHSHSHSHSHS TEST#################
                # self.is_delivery = True
                # self.planning_msg.mode = 'delivery1'

                ############################################SHSHSH###################################SSSHSHSHSHSH

                if self.is_delivery is True:
                    print(self.dmode)
                    print(self.sign_map)
                    if self.planning_msg.mode == "delivery1":

                        self.maxClassA, self.order_b, b_count = self.delivery_decision.detect_signs(self.object_msg.bounding_boxes)

                        self.delivery_decision.delivery_count(self.order_b, b_count)

                        self.target_b = self.delivery_decision.target_b_decision(self.maxClassA)

                        if not self.dmode == "pickup_complete":
                            self.sign_map = self.map_maker.a_sign_mapping(self, self.delivery_decision.delivery_path_a, self.obstacle_msg.circles)
                            for i, sign in self.sign_map.items():
                                if sign.index > 75 and sign.index < 86:
                                    continue
                                self.local_path = self.delivery_decision.path_plan(self.delivery_decision.delivery_path_a, sign.index)

                            if self.local_path.x and self.dmode != "pickup_stop":
                                self.dmode = "pickup"
                                self.planning_msg.mode = "pickup"
                                self.planning_msg.path = self.local_path

                                end_point_x = self.local_path.x[-1]
                                end_point_y = self.local_path.y[-1]

                                distance = hypot(end_point_x - self.local.x, end_point_y - self.local.y)
                                if distance < 1.5:
                                    self.planning_msg.mode = "normal_stop"

                                if self.serial_msg.speed < 0.1:
                                    self.dmode = "pickup_stop"
                                    self.count = time.time()

                            elif self.dmode == "pickup_stop":
                                print("--------------------")
                                self.planning_msg.mode = "normal_stop"

                                if time.time() - self.count > 3:
                                    self.dmode = "pickup_complete"

                    elif self.planning_msg.mode == "delivery2":
                        self.sign_map = self.map_maker.b_sign_mapping(self, self.delivery_decision.delivery_path_b, self.obstacle_msg.circles)

                        for i, sign in self.sign_map.items():
                            print(i, self.target_b)
                            if i == self.target_b:
                                self.local_path = self.delivery_decision.path_plan(self.delivery_decision.delivery_path_b, sign.index)

                        if self.local_path.x and self.dmode != "drop_stop":
                            self.dmode = "drop"
                            self.planning_msg.mode = "drop"
                            self.planning_msg.path = self.local_path

                            end_point_x = self.local_path.x[-1]
                            end_point_y = self.local_path.y[-1]

                            distance = hypot(end_point_x - self.local.x, end_point_y - self.local.y)
                            if distance < 1.5:
                                self.planning_msg.mode = "normal_stop"

                            if self.serial_msg.speed < 0.1:
                                self.dmode = "drop_stop"
                                self.count = time.time()

                        elif self.dmode == "drop_stop":
                            print("--------------------")
                            self.planning_msg.mode = "normal_stop"

                            if time.time() - self.count > 3:
                                self.dmode = "drop_complete"
                                self.is_delivery = False

                    #     if self.delivery_decision.stop_decision(self.planning_msg.path.x[-1], self.planning_msg.path.y[-1]):
                    #         self.planning_msg.mode = "delivery_stop"
                    #         self.count = time.time()

                    # elif self.planning_msg.mode == "delivery_stop" and time.time() - self.count > 5.5:
                    #     self.planning_msg.mode = "general"
                    #     self.is_delivery = False

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

                # self.target.points=[]
                # self.target.points.append(Point32(self.global_path.x[self.veh_index],self.global_path.y[self.veh_index],0))
                # self.target.header.stamp=rospy.Time.now()
                # self.target_pub.publish(self.target)

                # self.target.points=[]
                # self.target.points.append(self.planning_msg.point)
                # self.target.header.stamp=rospy.Time.now()
                # # self.target_pub.publish(self.target)

                self.map.points = self.map_maker.showObstacleMap().points
                self.map.header.stamp = rospy.Time.now()
                self.map_pub.publish(self.map)

                theta = radians(self.local.heading)
                self.obs.points = []
                for circle in self.obstacle_msg.circles:
                    # 장애물 절대좌표 변환
                    x = circle.center.x * cos(theta) + circle.center.y * -sin(theta) + self.local.x
                    y = circle.center.x * sin(theta) + circle.center.y * cos(theta) + self.local.y
                    self.obs.points.append(Point32(x, y, 0))
                self.obs.header.stamp = rospy.Time.now()
                self.obs_pub.publish(self.obs)

                self.pose.points = []
                self.pose.points.append(Point32(self.local.x, self.local.y, 0))
                self.pose.header.stamp = rospy.Time.now()
                self.pose_pub.publish(self.pose)

                self.planning_msg.cur_index = self.veh_index
                self.planning_info_pub.publish(self.planning_msg)
                # print(self.local.heading, self.global_path.heading[self.veh_index])
                rate.sleep()

    # Callback Functions
    def obstacleCallback(self, msg):
        self.obstacle_msg.segments = msg.segments
        self.obstacle_msg.circles = msg.circles
        self.obstacle_msg.circle_number = msg.circle_number

        self.map_maker.mapping(self, msg.circles)

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
        self.object_msg = msg

        # print(len(msg.bounding_boxes))
        # for box in msg.bounding_boxes:
        #     print(box.Class)
        # print(box.xmin)
        # print(box.xmax)

    def parkingCallback(self, msg):
        print("Parking Callback run")
        self.parking_target = msg.data


if __name__ == "__main__":
    planner = Planner()
    planner.run()
