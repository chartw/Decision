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

from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from std_msgs.msg import Float32, Time, String, Int16, Int32

# from lane_detection.msg import lane
from lib.planner_utils.global_path_plan import GPP
from lib.planner_utils.local_point_plan import LPP
from lib.planner_utils.mission_plan import MissionPlan
from lib.planner_utils.parking_path_plan import ParkingPlan
from lib.planner_utils.sig_int_handler import SigIntHandler

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

        self.mission_goal = Point32()
        self.target_map = {}
        self.sign_map = {}
        self.past_dist = 0
        self.past_min_dist = 0
        self.start_time = time.time()
        self.dynamic_flag = False
        self.check_veh_index_first = True

        self.control_ready = 0

        self.veh_index = 0
        self.target_index = 0
        self.stop_index = 0


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
        self.signal_ignore=False

###################################################################
        self.arg = rospy.myargv(argv=sys.argv)
        # arg[0] == planner.py
        self.mapname = str(self.arg[1])
        if self.mapname == "songdo" or self.mapname == "kcity":
            self.goal_node = str(self.arg[2])

        else:
            if len(self.arg) >= 4:
                self.parking_target = int(self.arg[3])
                self.veh_index = int(self.arg[2])
            elif len(self.arg) >= 3:
                self.veh_index = int(self.arg[2])
            else:
                self.veh_index = 0
###################################################################

        # gpp 변수 선언
        self.global_path_maker = GPP(self)
        self.local_path_maker = LPP(self)
        self.parking_planner = ParkingPlan(self)

        self.misson_planner = MissionPlan(self)

        self.vis_parking_path = PointCloud()
        self.vis_parking_path.header.frame_id = "world"

        self.vis_local_path = PointCloud()
        self.vis_local_path.header.frame_id = "world"

        self.vis_global_path = PointCloud()
        self.vis_global_path.header.frame_id = "world"

        self.map = PointCloud()
        self.map.header.frame_id = "world"

        self.pose = PointCloud()
        self.pose.header.frame_id = "world"

        self.target = PointCloud()
        self.target.header.frame_id = "world"




        self.planning_info_pub = rospy.Publisher("/planner", Planning_Info, queue_size=1)
        self.local_path_pub = rospy.Publisher("/local_path", PointCloud, queue_size=1)
        self.parking_path_pub = rospy.Publisher("/parking_path", PointCloud, queue_size=1)
        self.map_pub = rospy.Publisher("/map_pub", PointCloud, queue_size=1)
        self.pose_pub = rospy.Publisher("/pose_pub", PointCloud, queue_size=1)
        self.global_path_pub = rospy.Publisher("/global_path", PointCloud, queue_size=1)
        self.target_pub = rospy.Publisher("/target", PointCloud, queue_size=1)

        local_sub = message_filters.Subscriber("/pose", Odometry)

        # Localization
        rospy.Subscriber("/pose", Odometry, self.localCallback)
        rospy.Subscriber("/serial", Serial_Info, self.serialCallback)
        rospy.Subscriber("/control", Serial_Info, self.controlCallback)

    def run(self):
        rate = rospy.Rate(20)  # 100hz

        while not rospy.is_shutdown():
            print(len(self.local_path.x))
            if self.is_local:
                if self.gpp_requested:
                    self.global_path = self.global_path_maker.path_plan()
                    self.gpp_requested = False
                    if self.veh_index == 7000:
                        self.is_delivery = True

                if not self.control_ready:
                    self.planning_msg.mode = "general"
                    self.planning_msg.path = self.global_path
                    self.planning_info_pub.publish(self.planning_msg)
                    rate.sleep()
                    continue	

   					                # Localization Information
                self.planning_msg.local = self.local
                self.veh_index = self.get_veh_index()
                self.stop_index = self.stop_line_checker.stop_idx_check(planner)

                if self.planning_msg.mode == "parking":
                	self.is_parking = True

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

                self.map.header.stamp = rospy.Time.now()
                self.map_pub.publish(self.map)

                self.pose.points = []
                self.pose.points.append(Point32(self.local.x, self.local.y, 0))
                self.pose.header.stamp = rospy.Time.now()
                self.pose_pub.publish(self.pose)

                self.planning_msg.cur_index = self.veh_index

                self.planning_info_pub.publish(self.planning_msg)
                rate.sleep()

    def localCallback(self, msg):
        self.local.x = msg.pose.pose.position.x
        self.local.y = msg.pose.pose.position.y
        self.local.heading = msg.twist.twist.angular.z
        self.is_local = True

    def serialCallback(self, msg):
        self.serial_msg = msg

    def parkingCallback(self, msg):
        print("Parking Callback run")
        print(msg.data)
        if len(self.arg)==4:
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
