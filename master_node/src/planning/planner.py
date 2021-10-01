#!/usr/bin/env python3
# -*- coding:utf-8 -*-
import rospy

import numpy as np
from math import radians, degrees, sin, cos, hypot, atan2, pi
import sys
import time
import message_filters
from master_node.msg import Obstacles, Planning_Info, Path, Local, Serial_Info, CalibObjects
from nav_msgs.msg import Odometry

from yolov4_trt_ros.msg import Detector2DArray
from yolov4_trt_ros.msg import Detector2D

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
from lib.planner_utils.sig_int_handler import SigIntHandler

import signal


class Planner:
    def __init__(self):
        rospy.init_node("Planner", anonymous=False)

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
        # self.object_msg = BoundingBoxes()
        self.object_msg = Detector2DArray()
        self.surface_msg = String()
        self.serial_msg = Serial_Info()
        self.parking_msg = Int32()

        # data 변수 선언
        self.global_path = Path()
        self.local_path = Path()
        self.parking_path = Path()
        self.parking_backpath = Path()
        self.local = Local()
        self.obs_local = Local()
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

        self.control_ready = 0

        # self.veh_index = 0
        self.target_index = 0
        self.stop_index = 0

        # gpp 변수 선언
        self.global_path_maker = GPP(self)
        self.local_path_maker = LPP(self)
        self.parking_planner = ParkingPlan(self)

        self.misson_planner = MissionPlan(self)
        self.map_maker = Mapping()
        self.stop_line_checker = StopLine()
        self.traffic_light = trafficLight()
        self.delivery_decision = deliveryClass(self)

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
        self.parking_target_index = 0
        self.target_index = 0
        self.target_b = None
        self.del1_end_index = -1
        self.del2_end_index = -1

        self.is_delivery = False
        self.dmode = ""

        self.count = 0
        self.booleanFalse = False

        self.arg = rospy.myargv(argv=sys.argv)
        # arg[0] == planner.py
        self.mapname = str(self.arg[1])
        if self.mapname == "songdo" or self.mapname == "kcity":
            self.goal_node = str(self.arg[2])

        else:
            if len(self.arg) >= 3:
                self.veh_index = int(self.arg[2])

            elif len(self.arg) >= 4:
                self.parking_target = int(self.arg[3])

            else:
                self.veh_index = 0

        self.planning_info_pub = rospy.Publisher("/planner", Planning_Info, queue_size=1)
        self.local_path_pub = rospy.Publisher("/local_path", PointCloud, queue_size=1)
        self.parking_path_pub = rospy.Publisher("/parking_path", PointCloud, queue_size=1)
        self.map_pub = rospy.Publisher("/map_pub", PointCloud, queue_size=1)
        self.obs_pub = rospy.Publisher("/obs_pub", PointCloud, queue_size=1)
        self.pose_pub = rospy.Publisher("/pose_pub", PointCloud, queue_size=1)
        self.global_path_pub = rospy.Publisher("/global_path", PointCloud, queue_size=1)
        self.target_pub = rospy.Publisher("/target", PointCloud, queue_size=1)

        # LiDAR
        # Pose(Local), Obstacles Time Synchronize
        obs_sub = message_filters.Subscriber("/obstacles", Obstacles)
        local_sub = message_filters.Subscriber("/pose", Odometry)
        ts = message_filters.ApproximateTimeSynchronizer([obs_sub, local_sub], 10, 0.1, allow_headerless=True)
        ts.registerCallback(self.obstacleCallback)

        # Pose(Local), CalibObjects Time Synchronize
        calib_sub = message_filters.Subscriber("/calib_object", CalibObjects)
        ts = message_filters.ApproximateTimeSynchronizer([calib_sub, local_sub], 10, 0.1, allow_headerless=True)
        ts.registerCallback(self.calibObjectCallback)

        rospy.Subscriber("/Parking_num", Int32, self.parkingCallback)

        # Localization
        rospy.Subscriber("/pose", Odometry, self.localCallback)

        # Vision - Object
        # def objectCallback(self, msg): self.object_msg = msg
        rospy.Subscriber("/detections", Detector2DArray, self.objectCallback)
        # rospy.Subscribeer("/")

        rospy.Subscriber("/serial", Serial_Info, self.serialCallback)

        # Vision - Surface
        rospy.Subscriber("/surface", String, self.surfaceCallback)

        rospy.Subscriber("/control", Serial_Info, self.controlCallback)

        # 맵 이름이랑, 도착노드 system argument로 받아서 실행하자~
        # ex) python3 planner.py songdo 38
        # 후에는 roslaunch 파일로 바꾸면서 parameter 가져오도록 변경

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
        print(len(self.global_path.x))
        for i in range(max(self.veh_index - 100, 0), self.veh_index + 100):
            dis = hypot(self.global_path.x[i] - self.local.x, self.global_path.y[i] - self.local.y)
            if min_dis > dis or min_dis == -1:
                min_dis = dis
                min_idx = i

        return min_idx

    def run(self):
        rate = rospy.Rate(20)  # 100hz

        while not rospy.is_shutdown():
            if self.is_local:
                if self.gpp_requested:
                    self.global_path = self.global_path_maker.path_plan()
                    self.gpp_requested = False
                    if self.veh_index == 7000:
                        self.is_delivery = True

                    if self.mapname == "sd_del2":
                        self.is_delivery = True

                if not self.control_ready:
                    self.planning_msg.mode = "general"
                    self.planning_msg.path = self.global_path
                    self.planning_info_pub.publish(self.planning_msg)
                    rate.sleep()
                    continue

                # endcheck = False
                # Localization Information
                self.planning_msg.local = self.local
                self.veh_index = self.get_veh_index()
                self.stop_index = self.stop_line_checker.stop_idx_check(planner)

                if not self.is_parking:
                    self.planning_msg.dist = self.check_dist()
                    self.planning_msg.mode = self.misson_planner.decision(self)

                # if self.planning_msg.mode == "general" or self.planning_msg.mode == "kid" or self.planning_msg.mode == "bump":
                #     self.planning_msg.path = self.global_path

                if self.planning_msg.mode == "small" or self.planning_msg.mode == "big":

                    self.target_map = self.map_maker.make_target_map(self)
                    if self.target_map:
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
                    # self.planning_msg.mode = "general"

                    self.planning_msg.dist = (self.stop_index - self.veh_index) / 10
                    signal = self.traffic_light.run(self.object_msg)
                    print(signal)
                    if self.global_path.mission[self.stop_index] in signal:
                        self.planning_msg.mode = "general"
                    else:
                        self.planning_msg.mode = "normal_stop"

                #####Parking
                if self.is_parking is True:
                    self.pmode = self.parking_planner.parking_state_decision(self)

                    print("Current Mission: ", self.planning_msg.mode)

                    # if self.pmode == "parking-base1":
                    #     self.parking_path = self.parking_planner.make_parking_path(1)

                    if self.pmode == "parking_ready":
                        self.parking_path = self.parking_planner.make_parking_path(self.parking_target)
                        self.planning_msg.path = self.parking_path
                        self.local_path = self.parking_path

                        for i in range(len(self.parking_path.x)):
                            self.parking_backpath.x.insert(0, self.parking_path.x[i])
                            self.parking_backpath.y.insert(0, self.parking_path.y[i])
                            self.parking_backpath.heading.insert(0, self.parking_path.heading[i])

                        print("==========parking path created")
                        # print(self.parking_path)
                        self.vis_parking_path.points = []
                        for i in range(len(self.parking_path.x)):
                            self.vis_parking_path.points.append(Point32(self.parking_backpath.x[i], self.parking_backpath.y[i], 0))
                        self.vis_parking_path.header.stamp = rospy.Time.now()
                        self.parking_path_pub.publish(self.vis_parking_path)

                    elif self.pmode == "parking_start":
                        self.parking_target_index, self.planning_msg.point = self.parking_planner.point_plan(self.parking_path, 2)
                        # self.planning_msg.parking_path = self.parking_path
                        # self.planning_msg.path.x = self.parking_path.x
                        # self.planning_msg.path.y = self.parking_path.y

                    elif self.pmode == "parking_backward":
                        self.parking_target_index, self.planning_msg.point = self.parking_planner.point_plan(self.parking_backpath, 2)
                        self.planning_msg.path = self.parking_backpath
                        self.local_path = self.parking_backpath

                    elif self.pmode == "parking_end":
                        self.mission_ing = False
                        self.is_parking = False

                    self.planning_msg.mode = self.pmode

                #####################SHSHSSHSHSHSHSHSHSHSSHSSHSHSHSHSHS TEST#################
                # self.is_delivery = True
                # self.planning_msg.mode = 'delivery1'

                ############################################SHSHSH###################################SSSHSHSHSHSH

                if self.is_delivery is True:
                    # print(self.dmode)
                    # print(self.sign_map)

                    if self.planning_msg.mode == "delivery1":

                        if not self.dmode == "pickup_complete":
                            self.local_path = self.delivery_decision.delivery_path_a
                            self.planning_msg.path = self.local_path

                            for i, sign in self.map_maker.sign_map.items():
                                if sign.Class in ["A1", "A2", "A3"]:
                                    self.del1_end_index = sign.index
                                    self.target_b = sign.Class.replace("A", "B")

                            if self.del1_end_index != -1:

                                if self.dmode != "pickup_stop":
                                    self.dmode = "pickup"
                                    self.planning_msg.mode = "pickup"

                                    end_point_x = self.local_path.x[self.del1_end_index]
                                    end_point_y = self.local_path.y[self.del1_end_index]

                                    distance = hypot(end_point_x - self.local.x, end_point_y - self.local.y)

                                    self.planning_msg.mode = "delivery_stop"
                                    self.planning_msg.dist = distance

                                    if self.serial_msg.speed < 0.1 and distance < 1.5:
                                        self.dmode = "pickup_stop"
                                        self.count = time.time()

                                elif self.dmode == "pickup_stop":
                                    self.planning_msg.mode = "delivery_stop"

                                    if time.time() - self.count > 5.5:
                                        self.dmode = "pickup_complete"

                        else:
                            self.planning_msg.mode = "pickup_complete"

                    elif self.planning_msg.mode == "delivery2":
                        self.local_path = self.delivery_decision.delivery_path_b
                        self.planning_msg.path = self.local_path
                        print(self.target_b)
                        for i, sign in self.map_maker.sign_map.items():
                            if sign.Class in ["A1", "A2", "A3"]:
                                self.del1_end_index = sign.index
                                self.target_b = sign.Class.replace("A", "B")

                            if sign.Class == self.target_b:
                                self.del2_end_index = sign.index

                        if self.dmode != "drop_complete":
                            if self.del2_end_index != -1:
                                if self.dmode != "drop_stop":
                                    self.dmode = "drop"
                                    self.planning_msg.mode = "drop"

                                    end_point_x = self.local_path.x[self.del2_end_index]
                                    end_point_y = self.local_path.y[self.del2_end_index]

                                    distance = hypot(end_point_x - self.local.x, end_point_y - self.local.y)
                                    self.planning_msg.mode = "delivery_stop"
                                    self.planning_msg.dist = distance

                                    if self.serial_msg.speed < 0.1 and distance < 1.5:
                                        self.dmode = "drop_stop"
                                        self.count = time.time()

                                elif self.dmode == "drop_stop":
                                    self.planning_msg.mode = "delivery_stop"

                                    if time.time() - self.count > 5.5:
                                        self.dmode = "drop_complete"

                        elif self.dmode == "drop_complete":
                            self.planning_msg.mode == "drop_complete"

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
                if self.is_delivery == True:
                    self.map.points = self.map_maker.showSignMap().points
                else:
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
                # print(self.local.heading, self.global_path.heading[self.veh_index])

                # #@@@@@@@@@@@@@@@@@@@@@@배달stop test@@@@@@@@@@@@@@@@
                # # self.planning_msg.mode = 'general'

                # jj=230
                # distance = hypot(self.global_path.x[jj] - self.local.x, self.global_path.y[jj] - self.local.y)
                # print('dis',distance)
                # if distance < 4.1: #돌려보고 수정하기
                #     self.planning_msg.mode = "normal_stop"
                # #@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

                self.planning_info_pub.publish(self.planning_msg)
                rate.sleep()

    # Callback Functions
    def obstacleCallback(self, obs, local):
        self.obs_local.x = local.pose.pose.position.x
        self.obs_local.y = local.pose.pose.position.y
        self.obs_local.heading = local.twist.twist.angular.z

        self.obstacle_msg.segments = obs.segments
        self.obstacle_msg.circles = obs.circles
        self.obstacle_msg.circle_number = obs.circle_number
        if self.global_path.x:
            self.map_maker.mapping(self, obs.circles, self.obs_local)

    def calibObjectCallback(self, calib_object, local):
        calib_object_local = Local()
        calib_object_local.x = local.pose.pose.position.x
        calib_object_local.y = local.pose.pose.position.y
        calib_object_local.heading = local.twist.twist.angular.z

        if self.planning_msg.mode in ["delivery1", "delivery2"]:
            self.map_maker.delivery_sign_mapping(self.local_path, calib_object, calib_object_local)

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
        print(msg.data)
        if self.arg[4]:
            pass
        else:
            self.parking_target = msg.data

    def controlCallback(self, msg):
        self.control_ready = msg.ready


if __name__ == "__main__":

    SI = SigIntHandler()
    SI.run()
    planner = Planner()
    planner.run()
