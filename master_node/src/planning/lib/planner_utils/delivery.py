import numpy as np
from numpy import argmax, array
from geometry_msgs.msg import Point32
from math import hypot, degrees
import csv

from master_node.msg import Path

from lib.planner_utils.cubic_spline_planner import Spline2D
from yolov4_trt_ros.msg import Detector2DArray
from yolov4_trt_ros.msg import Detector2D

class deliveryClass:
    def __init__(self,planner):
        self.sign_count_a = {'A1':0, 'A2': 0, 'A3':0}
        #voting self.order_count_b = {'123':0, '132':0, '}
        self.sign_names = ['A1', 'A2', 'A3', 'B1', 'B2', 'B3']
        self.ratio_th = 1.5
        self.size_th = 2
        self.queue = []
        self.queue_size=10
        self.sign_size_b = {'B1':-1, 'B2': -1, 'B3':-1}
        self.delivery_path_a = Path()
        self.delivery_path_b = Path()
        self.b_compare = {'B1>B2':0, 'B2>B1':0, \
                        'B2>B3':0, 'B3>B2':0, \
                        'B3>B1':0, 'B1>B3':0}
        self.maxClassA = 'A2'
        self.max_count = 0
        self.make_delivery_path(planner)
        self.result_mapping={'0':'A1', '1':'A2', '2':'A3', '3':'B1', '4':'B2', '5':'B3', '6':' Red', '7':'Yellow', '8' : 'RedLeft', '9': 'GreenLeft', '10':'Green'}
        self.b_order_vote = {'123':0, '132': 0, '213':0, '231':0, '312':0, '321':0}
    def make_delivery_path(self,planner):
        temp=""
        if "sd" in planner.map:
            temp="sd_"
        with open("./map/kcity_map/Delivery_kcity/"+temp+"delivery_a.csv", mode="r") as csv_file:
            csv_reader = csv.reader(csv_file)
            for line in csv_reader:
                self.delivery_path_a.x.append(float(line[0]))
                self.delivery_path_a.y.append(float(line[1]))

                deg_yaw=(degrees(float(line[2]))+360) % 360
                self.delivery_path_a.heading.append(deg_yaw)

        with open("./map/kcity_map/Delivery_kcity/"+temp+"delivery_b.csv", mode="r") as csv_file:
            csv_reader = csv.reader(csv_file)
            for line in csv_reader:
                self.delivery_path_b.x.append(float(line[0]))
                self.delivery_path_b.y.append(float(line[1]))

                deg_yaw=(degrees(float(line[2]))+360) % 360
                self.delivery_path_b.heading.append(deg_yaw)

        return self.delivery_path_a, self.delivery_path_b
    

    def detect_signs(self, msg):
        detections = msg.detections
        
        b_count = 0
        for detection in detections:
            # print(detection)
            sign_id = str(detection.results.id)
            sign_name = self.result_mapping[sign_id]
            
            if sign_name in self.sign_names[0:3]: #A1~A3
                print(self.sign_count_a)
                self.sign_count_a[sign_name] += 1
                if self.max_count < self.sign_count_a[sign_name]:
                    self.max_count = self.sign_count_a[sign_name]
                    self.maxClassA = sign_name
             
            elif sign_name in self.sign_names[3:6]: #B1~B3
                b_count +=1
                x = detection.bbox.size_x
                y = detection.bbox.size_y
                self.sign_size_b[sign_name] = x*y


        sorted_signs_b = sorted(self.sign_size_b.items(), key=lambda x: x[1], reverse=True)
        order_b = []
        for i in range(3):
            order_b.append(sorted_signs_b[i][0])

        #초기화
        self.sign_size_b = {'B1':-1, 'B2': -1, 'B3':-1}
        
        return self.maxClassA, order_b, b_count

    def delivery_count(self, order_b, b_count):
        
        # print('order_b', order_b)
        # print('b_count', b_count)
        if b_count == 2:
            # result= order_b[0] + '>' + order_b[1]
            # self.b_compare[result] +=1
            for order in self.b_order_vote.keys():
                print('============================sibal', order, order_b)

                if order.find(order_b[0][1]) < order.find(order_b[1][1]):
                    print('==========')
                    self.b_order_vote[order] += 1

        elif b_count ==3:
            order = order_b[0][1] + order_b[1][1] + order_b[2][1]
            self.b_order_vote[order] += 1

        # print(self.b_compare)
        
    def target_b_decision(self, maxClassA):
        # print('maxA', maxClassA)
# 
        target_b = -1


        max_b_order = max(self.b_order_vote, key=self.b_order_vote.get)
        result = list(max_b_order)    
        
        print(self.b_order_vote)
        print('result', result)

        try:
            target_b = result.index(maxClassA[1])
        except:
            ValueError
        # print('target_b', target_b)
        print('target_b', target_b)
        return target_b


    def path_plan(self, path, sign_index):
        temp=Path()
        print(len(path.x), sign_index)
        x_list, y_list=[],[]
        x_list.append(path.x[0])
        y_list.append(path.y[0])
        x_list.append(path.x[sign_index])
        y_list.append(path.y[sign_index])

        csp=Spline2D(x_list, y_list)
        s=np.arange(0,csp.s[-1],0.1)
        rx, ry=[],[]
        for i_s in s:
            ix,iy=csp.calc_position(i_s)
            rx.append(ix)
            ry.append(iy)


        temp.x=rx
        temp.y=ry
        return temp

    def find_stop_point(self, targetPoint, slicedPath):
        min = 100
        min_x, min_y = 0

        for i in range(slicedPath):
            distance = hypot(slicedPath[i].x, slicedPath[i].y, targetPoint.x, targetPoint.y)
            if min > distance:
                min = distance
                min_x = slicedPath[i].x
                min_y = slicedPath[i].y

        return min_x, min_y


    def stop_decision(self, targetPoint):
        distance = hypot(self.local.x, self.local.y, targetPoint.x, targetPoint.y)
        return distance < 0.5

    def index_decision(self, order_b, sign_map, targetPoint):
        goal_order=order_b.index(targetPoint)

        return sign_map[goal_order].index()


