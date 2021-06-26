import pymap3d
#import csv
import rospy
# from nav_msgs.msg import Odometry
from master_node.msg import Local
from std_msgs.msg import String
# from obstacle_detector.msg import Obs
import numpy as np
from math import radians
from math import degrees
from math import sin
from math import cos
from math import hypot
from math import atan2
# import LPP
import sys
import serial
import struct
import matplotlib.pyplot as plt
from hybrid_a_star import path_plan 
from master_node.msg import Obstacles 
from master_node.msg import PangPang
from darknet_ros_msgs.msg import BoundingBoxes
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point
from master_node.msg import Dec

class Decision:
    def __init__(self):
        rospy.init_node('Decision', anonymous=False)
        self.pub = rospy.Publisher('/decision',Dec,queue_size=1)
        
        self.pubmsg=Dec()

        self.path =  
        self.lpath_x = []
        self.lpath_y = []
        #------------------------------------------------------
        self.gpaths=PointCloud()
        self.gpaths.header.frame_id='world'
        self.lpaths=PointCloud()
        self.lpaths.header.frame_id='world'
        self.cur_points=PointCloud()
        self.cur_points.header.frame_id = 'world'
        self.pub_gp=rospy.Publisher('/gpath',PointCloud,queue_size=1)
        self.pub_lp=rospy.Publisher('/lpath',PointCloud,queue_size=1)
        self.pub_c=rospy.Publisher('/cur_xy',PointCloud,queue_size=1)
        #------------------------------------------------------

        self.goal_x = 0
        self.goal_y = 0
        self.obx = []
        self.oby = []
        self.global_target_index =0
        self.local_target_index = 0
        self.goal_index = 0
        self.ob_num = 0

        self.center = []
        self.goal_point = '9999'
        self.first = True # ground station이랑 최초인지 확인

        # self.next = False
        self.start_idx = 0
        self.ser = serial.Serial('/dev/ttyUSB0',115200) # USB 권한 주
        self.break_once = False
        self.person_detect = 0
        self.animation_cnt = 0
        self.old_lpath_x = []
        self.old_lpath_y = []
        self.mode_sw = GlobalPath
        self.LPP_is_done = 0
        self.GPP_is_done = 0
        self.old_obstacle = 0


        
        # rospy.Rate(1000)def calc_dis(self, nx, ny):
        # print(nx, ny, )
    def calc_dis(self, nx, ny):
        distance = ((nx - self.pubmsg.pose.x)**2 +  (ny - self.pubmsg.pose.y)**2)**0.5

        return distance

    def LiDARCallback(self, msg):
            
        self.ob_num = msg.circle_number
        diff = self.ob_num - self.old_obstacle
        self.old_obstacle = self.ob_num

        self.obx, self.oby = [], []
        # print("gaesu" , len(msg.points))
        for pt in msg.points:
            self.obx.append(pt.x)
            self.oby.append(pt.y)

        if diff > 0:
            if self.LPP_is_done is 0:
                # self.control_data['target_idx']
                self.local_target_index = 0
                self.goal_index, ox, oy = self.makeObstacleMap(self.pubmsg.pose.x, self.pubmsg.pose.y, self.pubmsg.pose.yaw)
                self.lpath_x, self.lpath_y = path_plan(self.pubmsg.pose.x, self.pubmsg.pose.y, self.pubmsg.pose.yaw, self.path_x[self.goal_index], self.path_y[self.goal_index], self.path_yaw[self.goal_index], ox, oy, self.animation_cnt)
                self.mode_sw = LocalPath
                if len(self.lpath_x) is not 0:
                    self.LPP_is_done = 1

            elif self.mode_sw is LocalPath:
                get_test = []
                for i in range(int(self.ob_num)):
                    get_test.append(self.calc_dis(self.obx[i], self.oby[i]))
                ox, oy = [], []

                if min(get_test) > 2:
                    self.goal_index, ox, oy = self.makeObstacleMap(self.pubmsg.pose.x, self.pubmsg.pose.y, self.pubmsg.pose.yaw)
                    self.lpath_x, self.lpath_y = path_plan(self.pubmsg.pose.x, self.pubmsg.pose.y, self.pubmsg.pose.yaw, self.path_x[self.goal_index], self.path_y[self.goal_index], self.path_yaw[self.goal_index], ox, oy, self.animation_cnt)
            #--------------------------------------------------------------------------------
            for i in range(len(self.lpath_x)):
                lpath=Point32()
                lpath.x=self.lpath_x[i]
                lpath.y=self.lpath_y[i]
                self.lpaths.points.append(lpath)
            self.lpaths.header.stamp=rospy.Time.now()
            self.pub_lp.publish(self.lpaths)
            print('lpaths published.')
            self.lpaths.points = []
            #--------------------------------------------------------------------------------

    def select_start_node(self, cx, cy, cyaw):
        nodelist = []
        nodelist = makeTarget.node_set()
        # print(nodelist)
        min_dis = 99999
        min_idx = 10000
        temp_idx = 10000
        temp_dis = 9999

        # valid_node_list = []
        for node in nodelist:
            # dx, dy = nodelist[node].x - cx, nodelist[node].y - cy
            # dyaw = (atan2(dy, dx) *180 / 3.14) - 90
            # if dyaw < 0:
            #     dyaw += 360
            # print(node, dyaw, cyaw)

            # if abs(dyaw - cyaw) < 10:
            #     valid_node_list.append(nodelist[node])
            temp_dis = self.calc_dis(nodelist[node].x, nodelist[node].y)
            if temp_dis < min_dis:
                min_dis = temp_dis
                min_idx = node

        return min_idx

    def ground(self, msg):
        print('from goal_node', msg.data)
        self.first = False
        self.goal_point = msg.data

    def GPP(self):
        # print('from goal_node:', msg.data)
        self.global_target_index = 0
        self.path_x, self.path_y = [], []

        self.start_idx = self.select_start_node(self.pubmsg.pose.x, self.pubmsg.pose.y, self.pubmsg.pose.heading)
        print(self.pubmsg.pose.x, self.pubmsg.pose.y)
        # if self.goal_point is not '9999':
        self.path_x, self.path_y, self.path_yaw = makeTarget.path_connect(str(self.start_idx), '38') # 출발노드, 도착노드

        self.goal_x, self.goal_y = self.path_x[-1], self.path_y[-1]

        #--------------------------------------------------------------------------------
        for i in range(len(self.path_x)):
            gpath=Point32()
            gpath.x=self.path_x[i]
            gpath.y=self.path_y[i]
            self.gpaths.points.append(gpath)
        self.gpaths.header.stamp=rospy.Time.now()
        self.pub_gp.publish(self.gpaths)
        print('gpaths published.')
        #-------------------------------------------------------------

    def select_goal_pt(self, ox, oy, c_yaw):

        tmpidx = self.global_target_index 
        # print(tmpidx)

        while True:
            tmp_x = self.path_x[tmpidx]
            tmp_y = self.path_y[tmpidx]
            vec1 = (ox[1] - tmp_x, oy[1]- tmp_y)
            vec2 = (ox[0] - tmp_x, oy[0] - tmp_y)
            vec3 = (ox[2] - tmp_x, oy[2] - tmp_y)
            # vec4 = (ox[1] - tmp_x, oy[1] - tmp_y)
            # np.c
            tmp1 = np.cross(vec2, (cos(np.deg2rad(c_yaw)), sin(np.deg2rad(c_yaw))))
            tmp2 = np.cross(vec1, (cos(np.deg2rad(c_yaw)), sin(np.deg2rad(c_yaw))))
            tmp3 = np.cross(vec2, (cos(np.deg2rad(c_yaw - 90)), sin(np.deg2rad(c_yaw - 90))))
            tmp4 = np.cross(vec3, (cos(np.deg2rad(c_yaw - 90)), sin(np.deg2rad(c_yaw - 90))))

            # print("##", tmp1, tmp2, tmp3, tmp4)

            if tmp1 * tmp2 < 0 and tmp3 * tmp4 < 0:
                tmpidx += 1
            else:
                tmpidx -= 1
                return tmpidx -1



        # v_y = ox[30] - tmp_y


    def makeObstacleMap(self, c_x, c_y, c_yaw):
        ox, oy = [], [] # lidar, vision에서 받기

        ox.append(c_x-10)
        oy.append(c_y-1.0)
        ox.append(c_x+10)
        oy.append(c_y-1.0)

        ox.append(c_x-15)
        oy.append(c_y+26.0)
        ox.append(c_x+15)
        oy.append(c_y+26.0)
        

        for i in range(100):
            ox.append(c_x-10 + 0.2* i)
            oy.append(c_y - 1.0)


        for i in range(len(ox)):
            ox[i] -= c_x
            oy[i] -= c_y

        ox_rot, oy_rot = [], []
        rotate_yaw = c_yaw-90 % 360
        for i in range(len(ox)):
            ox_rot.append(ox[i]*cos(np.deg2rad(rotate_yaw)) + oy[i]*-sin(np.deg2rad(rotate_yaw)))
            oy_rot.append(ox[i]*sin(np.deg2rad(rotate_yaw)) + oy[i]*cos(np.deg2rad(rotate_yaw)))

        for i in range(len(ox)):
            ox_rot[i] += c_x
            oy_rot[i] += c_y

        for i in range(len(self.obx)):
            ox_rot.append(self.obx[i])
            oy_rot.append(self.oby[i])

        # print("#goal_index: ", self.control_data['target_idx'] + 200, self.select_goal(c_x, c_y, c_yaw, ox, oy))
        # goal_index = self.select_goal_pt(ox_rot, oy_rot, c_yaw)
        goal_index = self.global_target_index + 150
        # print("##############################################################", goal_index)
        return goal_index, ox_rot, oy_rot


        
    def LocalCallback(self,  msg):

        self.pubmsg.pose.heading  = msg.heading #imu 정북 기준으로 시계로 돌아갈때(시뮬)
        
        self.pubmsg.pose.x, self.pubmsg.pose.y  = msg.x, msg.y 

        #--------------------------------------------------------
        cur_point = Point32()
        cur_point.x = self.pubmsg.pose.x
        cur_point.y = self.pubmsg.pose.y
        self.cur_points.points.append(cur_point)
        self.cur_points.header.stamp = rospy.Time.now()
        self.pub_c.publish(self.cur_points)
        #--------------------------------------------------------
        if self.GPP_is_done is 0:
            print("GPP start")
            self.GPP()
            # self.first = False
            self.mode_sw = GlobalPath
            self.GPP_is_done = 1

        # 골포인트 성공할시
        if self.calc_dis(self.path_x[self.goal_index], self.path_y[self.goal_index]) < 4 and self.mode_sw is LocalPath:
            self.mode_sw = GlobalPath
            self.LPP_is_done = 0
            self.old_obstacle = 0
            
        if mode_sw = GlobalPath:
            self.pubmsg.path

    def VisionCallback(self,msg):
        self.person_detect = 0
        for os in msg.bounding_boxes:
            if(os.Class == "PERSON" and self.ob_num > 0):
                self.person_detect = 1
                # self.control_data['target_speed'] = 0x00
                print("person is detected")
    

dec = Decision()
rospy.Subscriber('/local',Local,dec.LocalCallback)
rospy.Subscriber('/pangpang',PangPang,dec.LiDARCallback)
rospy.Subscriber('/darknet_ros/bounding_boxes_c',BoundingBoxes, VisionCallback)
rate = rospy.Rate(500)
rospy.spin()
