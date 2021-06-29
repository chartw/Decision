# -*- coding:utf-8 -*-
import rospy


import numpy as np
from math import radians, degrees, sin, cos, hypot, atan2, pi
import sys
import time
from hybrid_a_star import path_plan
from master_node.msg import Obstacles, PangPang, Local, Path
from nav_msgs.msg import Odometry
from darknet_ros_msgs.msg import BoundingBoxes
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from std_msgs.msg import Float32, Time, String

# from lane_detection.msg import lane
from map import global_path_plan

# class Path():
#     def __init__(self):
#         self.x=[]
#         self.y=[]
#         self.yaw=[]

# 이걸 메세지 타입으로 만들자~ 아님 말구
class Pose:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.yaw = 0


class Planner:
    def __init__(self):
        rospy.init_node("Planner", anonymous=False)

        # 맵 이름이랑, 도착노드 system argument로 받아서 실행하자~
        # ex) python3 planner.py songdo 38
        arg = rospy.myargv(argv=sys.argv)
        self.map = arg[1]
        self.goal_node = arg[2]
        self.start_node = None

        # publish 정의
        global_path_pub = rospy.Publisher("/global_path", Path, queue_size=1)
        local_target_pub = rospy.Publisher("/local_target", Point32, queue_size=1)
        local_path_pub = rospy.Publisher("/local_path", Path, queue_size=1)
        mission_mode_pub = rospy.Publisher("/mission_mode", String, queue_size=1)

        # subscriber 정의
        # LiDAR
        rospy.Subscriber("/obstacles", Obstacles, self.obstacleCallback)
        # Localization
        rospy.Subscriber("/pose", Odometry, self.positionCallback)
        # Vision - Object
        rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, self.objectCallback)

        # 상태 flag
        self.is_position = False
        self.is_obstacle = False
        self.is_object = False
        self.gpp_requested = True
        self.is_global_path_pub = False

        # gpp 변수 선언
        path_maker = global_path_plan.GPP(self.map)

        # data 변수 선언
        self.global_path = Path()
        self.obstacle = Obstacles()
        self.position = Pose()
        self.is_person = False

        rate = rospy.Rate(100)  # 100hz

        while not rospy.is_shutdown():
            # gpp가 필요하고, 위치 정보가 들어와 있을 때 gpp 실행
            if self.is_position and self.gpp_requested:
                self.start_node = self.select_start_node()
                self.global_path = path_maker.path_connect(self.start_node, self.goal_node)
                self.gpp_requested = False
                self.is_global_path_pub = True

            # global path가 생성되어 새로 publish 해야 할때
            if self.is_global_path_pub:
                global_path_pub.publish(self.global_path)
                self.is_global_path_pub = False
            rate.sleep()

    # 가장 가까운 노드를 시작 노드로 설정
    def select_start_node(self, path_maker):
        nodelist = path_maker.nodelist

        min_dis = 99999
        min_idx = 10000
        temp_idx = 10000
        temp_dis = 9999

        for node in nodelist:
            temp_dis = self.calc_dis(nodelist[node].x, nodelist[node].y)
            if temp_dis < min_dis:
                min_dis = temp_dis
                min_idx = node

        return min_idx

    def calc_dis(self, nx, ny):
        distance = ((nx - self.position.x ** 2) + (ny - self.position.y) ** 2) ** 0.5

        return distance

    def positionCallback(self, msg):
        self.position.x = msg.pose.pose.position.x
        self.position.y = msg.pose.pose.position.y
        self.position.yaw = msg.twist.twist.angular.z
        self.is_position = True

    def obstacleCallback(self, msg):
        self.obstacle_msg = msg

    def objectCallback(self, msg):
        self.object_msg = msg


if __name__ == "__main__":
    planner = Planner()
    planner.run()
