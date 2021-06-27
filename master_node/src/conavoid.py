#-*- coding:utf-8 -*-

'''
self.control_data 는 실시간으로 master.py, sender.py와 공유됨 
'''

import pymap3d
#import csv
import rospy
from nav_msgs.msg import Odometry
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
from math import pi
# import LPP
import sys
import serial
import struct
import time
import message_filters
import matplotlib.pyplot as plt
from hybrid_a_star import path_plan 
from master_node.msg import Obstacles 
from master_node.msg import PangPang
from darknet_ros_msgs.msg import BoundingBoxes
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from std_msgs.msg import Float32
from std_msgs.msg import Time
from lane_detection.msg import lane


map="kcity_"


# sys.path.append("./map/")

# try:
#     import GPP
# except:
#     raise

sys.path.append("./map1/")

try:
    import makeTarget
except:
    raise
class Path():
    def __init__(self):
        self.x=[]
        self.y=[]
        self.yaw=[]
class Pose():
    def __init__(self):
        self.x=0
        self.y=0
        self.yaw=0
        self.update=False

class PID():
    def __init__(self):
        self.Kp_ld = 0.03

        self.Kp_v = 50
        self.Ki_v = 5
        self.Kd_v = 10

        self.Kp_s = 0.5
        self.Ki_s = 0.01
        self.Kd_s = 0

        self.speed_look_ahead = 6
        self.V_err = 0
        self.V_err_old = 0
        self.V_in = 0
        self.V_veh = 0
        self.V_err_pro = 0
        self.V_err_inte = 0
        self.V_err_deri = 0

        self.steering_veh = 0
        self.steering_err =0
        self.steering_err_old = 0
        self.steering_in = 0
        self.steering_err_inte = 0
        self.steering_err_deri = 0


class Controller:
    def __init__(self):
        rospy.init_node('Decision', anonymous=False)
        self.ser = serial.Serial('/dev/ttyUSB0',115200) # USB 권한 주
        self.look_ahead=4
        self.WB=1
        self.target_speed=80
        self.path=Path()
        self.pose=Pose()
        self.pid=PID()

        self.goal=Point32()
        self.avoid_target=Point32()
        self.avoid_goal=Point32()
        self.lane_target=Point32()

        self.target_idx =0
        self.start_idx = 0
        self.target_steer=0
        self.mode_sw = "general"
 
        self.person_detect = 0
        self.obstacle_detect=0
        
        # Displacement - encoder
        self.msg = Float32()
        self.pub_dis = rospy.Publisher('/Displacement', Float32, queue_size=1)
        self.pre_add = 0 #엔코더에서 사용하는 저장변수
        self.now_add = 0 #엔코더에서 사용하는 저장변수
        self.enc_flag = 0 #엔코더에서 사용하는 flag

        self.pub_test=rospy.Publisher('/decision',String,queue_size=1)

        self.flagtimeminkyu = time.time()

    
    def select_target(self, cx, cy, cidx, path_x, path_y):
        valid_idx_list = []

        for i in range(cidx, len(path_x)):
            dis = ((path_x[i]-cx)**2 +(path_y[i]-cy)**2)**0.5

            if dis <= self.look_ahead:
                valid_idx_list.append(i)
            if len(valid_idx_list) != 0 and dis > self.look_ahead:
                break
        if len(valid_idx_list) == 0:
            return 0
        else:
            return valid_idx_list[len(valid_idx_list) - 1]

             
    def cal_steering(self, cur_x, cur_y, cur_yaw, look_ahead):
        if len(self.path.x) is not 0:
            self.target_idx = self.select_target(cur_x, cur_y, self.target_idx, self.path.x, self.path.y)

            if self.mode_sw is "general":
                target_x = self.path.x[self.target_idx]
                target_y = self.path.y[self.target_idx]
            elif self.mode_sw is "avoidance":
                target_x = self.avoid_target.x
                target_y = self.avoid_target.y
                # target_x = self.lpath_x[10]
                # target_y = self.lpath_y[10]
            
            return self.steering_angle(cur_x, cur_y, cur_yaw, target_x, target_y)
        return 0
             
    def steering_angle(self,  cur_x, cur_y, cur_yaw, target_x, target_y):
        # pure pursuit 계산되는 부분 
        if self.mode_sw is "general":
            tmp_th = degrees(atan2((target_y - cur_y), (target_x - cur_x)))

            tmp_th = tmp_th%360

            alpha =  cur_yaw - tmp_th  
            # print('alpha:',  alpha)
            # print('tmp_th', tmp_th)
            if abs(alpha)>180:
                if (alpha < 0) :
                    alpha += 360
                else :
                    alpha -= 360
            # print('alpha2:',  alpha)        
            alpha = max(alpha,  -90)
            alpha = min(alpha,  90)
            # print('modified alpha:',  alpha)
            # alpha = -alpha
            delta = degrees(atan2(2*self.WB*sin(radians(alpha))/self.look_ahead,1))
            # print('delta:',  delta)
            
            #test = tmp_th - cur_yaw
            
            
            if abs(delta)>180:
                if (delta < 0) :
                    delta += 360
                else :
                    delta -= 360
            
            
            # print("delta2 : ", delta)
            if abs(delta)>30:
                if delta > 0:
                    # print('del;',  '1900')
                    return 1999
                else :
                    # print('del;',  '-1900' )
                    return -1999
            else :
                delta = 71*delta
                # print('del:',  delta)
                return int(delta)
        elif self.mode_sw is "avoidance":
            cur_x, cur_y, cur_yaw = 0, 0, 0
            tmp_th = degrees(atan2((target_y - cur_y), (target_x - cur_x)))
            tmp_th = tmp_th%360
            # print("temp", tmp_th)
            alpha =  cur_yaw - tmp_th
            # alpha = 360 - alpha
            # print("what?: ", alpha)
            if abs(alpha)>180:
                if (alpha < 0) :
                    alpha += 360
                else :
                    alpha -= 360
            # print("what2?: ", alpha)
            alpha = max(alpha,  -90)
            alpha = min(alpha,  90)
            # print("alpha", alpha)
            # delta = degrees(atan2(2*self.control_data['WB']*sin(radians(alpha))/1,1))
            # print("delta :", delta)
            delta = alpha
            # if target_y>0:
            #     delta = -alpha
            # print(delta)
            if abs(delta)>180:
                if (delta < 0) :
                    delta += 360
                else :
                    delta -= 360
            # print("delta", delta)
            if abs(delta)>30:
                if delta > 0:
                    return 1999
                else :
                    return -1999
            else :
                delta = 71*delta

            
            return int(delta)

    def calc_dis(self, nx, ny):
        # print(nx, ny, )
        distance = ((nx - self.pose.x)**2 +  (ny - self.pose.y)**2)**0.5

        return distance

    def getAvoidGoal(self, msg):

        clstob=msg.segments[0]
        for seg in msg.segments:
            if seg.last_point.x<seg.first_point.x:
                temp=seg.last_point
                seg.last_point=seg.first_point
                seg.first_point=temp
            
            if self.calc_dis(clstob.first_point.x,clstob.first_point.y) > self.calc_dis(seg.first_point.x,seg.first_point.y):
                clstob=seg

        if clstob.first_point.x < 1:
            return 1, 0
        d=1.5
        rad=np.arctan2(clstob.last_point.y - clstob.first_point.y, clstob.last_point.x - clstob.first_point.x)
        # print(rad)
        return clstob.last_point.x + (d*cos(rad)), clstob.last_point.y + (d*sin(rad))

    def line_detect(self, point):
        x_sum = 0
        y_sum = 0
        number = len(point)
        for i in range(number):
            x_sum += point[i].x
            y_sum += point[i].y
        number = float(number)
        x_mean = x_sum / number
        y_mean = y_sum / number

        up = 0
        down = 0

        for i in range(int(number)):
            up += (point[i].x-x_mean) * (point[i].y - y_mean)
            down += (point[i].x-x_mean) ** 2

        a = up / down
        b = y_mean - a * x_mean

        return a, b

    def getlane(self, msg):
        self.lane_curv = msg.curvature

        left_first = Point32()
        left_last = Point32()
        right_first = Point32()
        right_last = Point32()
        center_first = Point32()
        center_last = Point32()

        left_x=[]
        left_y=[]
        right_x=[]
        right_y=[]

        for i in range(len(msg.left)):
            left_x.append(msg.left[i].x)
            left_y.append(msg.left[i].y)

        for i in range(len(msg.right)):
            right_x.append(msg.right[i].x)
            right_y.append(msg.right[i].y)

        if len(msg.left) != 0:
            l_a, l_b = self.line_detect(msg.left)
            left_first.x = min(left_x)
            left_first.y = min(left_y) * l_a + l_b
            left_last.x = max(left_x)
            left_last.y = max(left_y) * l_a + l_b
        else:
            l_a, l_b = 0, 0
            left_first.x = 0
            left_first.y = 0
            left_last.x = 0
            left_last.y = 0

        # right is not empty
        if len(msg.right) != 0:
            r_a, r_b = self.line_detect(msg.right)
            right_first.x = min(right_x)
            right_first.y = min(right_y) * r_a + r_b
            right_last.x = max(right_x)
            right_last.y = max(right_y) * r_a + r_b
        else:
            r_a, r_b = 0, 0
            right_first.x = 0
            right_first.y = 0
            right_last.x = 0
            right_last.y = 0


        c_a = (l_a + r_a) / 2
        c_b = 0.0
        

        center_first.x = 0.0
        center_first.y = 0.0
        center_last.x = 2.0
        center_last.y = 2.0 * c_a + c_b
        d=1.5
        l_rad=np.arctan2(left_last.y - left_first.y, left_last.x - left_first.x) - pi / 2
        r_rad=np.arctan2(right_last.y - right_first.y, right_last.x - right_first.x) +pi / 2
        if len(msg.left)!= 0 and len(msg.right) != 0:
            target_x, target_y = (left_last.x + right_last.x)/2 , (left_last.y + right_last.y)/2
        elif len(msg.left)== 0 and len(msg.right) == 0:
            target_x, target_y = 1, 0
        elif len(msg.left) != 0 and len(msg.right) == 0:
            print("left_deg : ",np.rad2deg(rad2))
            target_x = left_last.x + (d*cos(l_rad))
            target_y = left_last.y + (d*sin(l_rad))

        elif len(msg.left) == 0 and len(msg.right) != 0:
            print("right_deg : ",np.rad2deg(r_rad))

            target_x = right_last.x + (d*cos(r_rad))
            target_y = right_last.y + (d*sin(r_rad))

        return target_x, target_y

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
            temp_dis = self.calc_dis(nodelist[node].x, nodelist[node].y)
            if temp_dis < min_dis:
                min_dis = temp_dis
                min_idx = node

        return min_idx

    def GPP(self):
        # print('from goal_node:', msg.data)
        self.target_idx = 0
        self.path.x, self.path.y = [], []
        self.first = False

        self.start_idx = self.select_start_node(self.pose.x, self.pose.y, self.pose.yaw)
        print(self.pose.x, self.pose.y, self.pose.yaw)
        # if self.goal_point is not '9999':
        self.path.x, self.path.y, self.path.yaw = makeTarget.path_connect(str(self.start_idx), '18') # 출발노드, 도착노드

        self.goal.x, self.goal.y = self.path.x[-1], self.path.y[-1]


    def decision(self, msg):

        if self.pose.update is True and len(self.path.x) is 0:
            print("GPP start")
            self.GPP()
            self.mode_sw = "general"

        if self.mode_sw is "general":
            # print(""general"")
            self.target_speed = 80
        elif self.mode_sw is "avoidance":
            # print("avoidance")
            self.target_speed = 50         

    # Serial
    #----------------------------------------------        
        cnt=0x00

        result = self.ser.readline()
        self.ser.flushInput()
        # print(result)
        # print(result[0])

        if (result[0] is 0x53 and result[1] is 0x54 and result[2] is 0x58):
            res_arr = []
            res_idx = 0
            # print('okokok')

            while True:
                for i in range(len(result)):
                    if result[i] is 0x0A and i is not 17:
                        # print("### 0x0A Found!", i, "th data")
                        res_arr.append(0x0B)
                    else :
                        res_arr.append(result[i])

                if len(res_arr) < 18:
                    result = self.ser.readline()
                else:
                    break

            cnt = int(res_arr[15])
            self.V_veh = int(res_arr[6])
            # print(self.V_veh)
            # self.steering_veh = int(ord(res_arr[8])) 

            self.target_steer  = self.cal_steering(self.pose.x, self.pose.y,  self.pose.yaw, self.look_ahead)
            # print("res_arr is", res_arr[6])
            if self.V_veh < 50:
                self.target_speed = 200
            else:
                self.target_speed = 80
            self.serWrite(int(self.target_speed), int(self.target_steer), cnt)
            
            a = res_arr[11]
            b = res_arr[12]
            c = res_arr[13]
            d = res_arr[14]

            # print("I'm here" , a)
            add = (float(a + 256*b + 256**2*c + 256**3*d))
            # if self.enc_flag == 0:
            #     self.pre_add = add
            #     self.now_add = add
            #     self.enc_flag = 1
            # else:
            #     self.pre_add = self.now_add
            #     self.now_add = add

            # difference = self.now_add - self.pre_add
            
        

            self.msg = add

            self.pub_dis.publish(self.msg)
            self.pub_test.publish("decision")
            


    #----------------------------------------------

    def sensorCallback(self,lidar_msg,pos_msg):
        # lidar_stamp=time.strftime('%Y-%m-%d %H:%M:%S',time.localtime(lidar_msg.header.stamp.secs))+".%09d" % lidar_msg.header.stamp.nsecs
        # pos_stamp=time.strftime('%Y-%m-%d %H:%M:%S',time.localtime(pos_msg.header.stamp.secs))+".%09d" % pos_msg.header.stamp.nsecs
        # lane_stamp=time.strftime('%Y-%m-%d %H:%M:%S',time.localtime(lane_msg.header.stamp.secs))+".%09d" % pos_msg.header.stamp.nsecs
        # print("lidar stamp : \t", lidar_stamp)
        # print("pos stamp : \t",pos_stamp)
        # print("lane stamp : \t",lane_stamp)
        # print("==============================================")

        self.pose.yaw  = pos_msg.twist.twist.angular.z #imu 정북 기준으로 시계로 돌아갈때(시뮬)
        self.pose.x, self.pose.y  = pos_msg.pose.pose.position.x, pos_msg.pose.pose.position.y
        self.pose.update=True
        
        if len(lidar_msg.segments) is not 0:
            self.obstacle_detect=1
            self.avoid_target.x, self.avoid_target.y=self.getAvoidGoal(lidar_msg)
            # print(self.avoid_target)
            self.mode_sw="avoidance"
            theta=self.pose.yaw*pi/180
            self.avoid_goal.x=self.avoid_target.x*cos(theta)+self.avoid_target.y*-sin(theta) + self.pose.x
            self.avoid_goal.y=self.avoid_target.x*sin(theta)+self.avoid_target.y*cos(theta) + self.pose.y
        else:
            self.obstacle_detect=0
            self.mode_sw="general"
            self.avoid_target.x=0
            self.avoid_target.y=0
            
        
        if abs(self.flagtimeminkyu - time.time()) > 0.2 :
            self.person_detect = 0

        # pflag=0
        # for obj in obj_msg.bounding_boxes:
        #     if obj.Class == 'person' and self.obstacle_detect is 1 and int(obj.ymax)-int(obj.ymin) == 300:
        #         pflag=1
        
        # if pflag is 1:
        #     self.person_detect=1
        # else:
        #     self.person_detect=0
            
        # self.lane_target.x, self.lane_target.y = self.getlane(lane_msg)





        

    def getObstacleClass(self,msg):
        self.person_detect = 0
        for os in msg.bounding_boxes:
            if(os.Class == "person" and int(os.ymax)-int(os.ymin) >= 300):
                self.person_detect = 1
                # self.target_speed = 0x00
                print("person is detected")


        self.flagtimeminkyu = time.time()
        

print("Controller ON")
#print(self.control_data)
#print('controller')
# rospy.Subscriber("/vehicle_node", String, self.final)
# print("run ok")

# rospy.Subscriber('/goal_node', String, self.ground)
# if self.first == False:
# print('okokok')
# rospy.Subscriber('/uturn', uuu, self.uTurn)
con=Controller()
lidar=message_filters.Subscriber("/obstacles",Obstacles)
pos=message_filters.Subscriber("/pose",Odometry)
# lane=message_filters.Subscriber("/laneinformation", lane)
# obj=message_filters.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes)


ats=message_filters.ApproximateTimeSynchronizer([lidar, pos],10,0.1,allow_headerless=True)
ats.registerCallback(con.sensorCallback)

# rospy.Subscriber("/obstacles", Obstacles, self.getObstacles)
# rospy.Subscriber("/pose", Odometry, self.getOdoMsg)
rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, con.getObstacleClass)

rospy.Subscriber("/timer",Time,con.decision)
rospy.spin()


