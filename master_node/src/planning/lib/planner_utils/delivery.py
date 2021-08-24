import numpy as np
from numpy import argmax, array
from geometry_msgs.msg import Point32
from math import hypot, degrees
import csv

from master_node.msg import Path

from lib.planner_utils.cubic_spline_planner import Spline2D

class deliveryClass:
    def __init__(self):
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
        self.make_delivery_path()



    def make_delivery_path(self):
        with open("./map/kcity_map/Delivery_kcity/delivery_a.csv", mode="r") as csv_file:
            csv_reader = csv.reader(csv_file)
            for line in csv_reader:
                self.delivery_path_a.x.append(float(line[0]))
                self.delivery_path_a.y.append(float(line[1]))

                deg_yaw=(degrees(float(line[2]))+360) % 360
                self.delivery_path_a.heading.append(deg_yaw)

        with open("./map/kcity_map/Delivery_kcity/delivery_b.csv", mode="r") as csv_file:
            csv_reader = csv.reader(csv_file)
            for line in csv_reader:
                self.delivery_path_b.x.append(float(line[0]))
                self.delivery_path_b.y.append(float(line[1]))

                deg_yaw=(degrees(float(line[2]))+360) % 360
                self.delivery_path_b.heading.append(deg_yaw)

        return self.delivery_path_a, self.delivery_path_b

    def detect_signs(self, boxes):
        b_count = 0
        for i in range(len(boxes)):
            box = boxes[i]
            if box.Class in self.sign_names[0:3]: #A1~A3
                print(self.sign_count_a)
                self.sign_count_a[box.Class] += 1
                if self.max_count < self.sign_count_a[box.Class]:
                    self.max_count = self.sign_count_a[box.Class]
                    self.maxClassA = box.Class
             
            elif box.Class in self.sign_names[3:6]: #B1~B3
                b_count +=1
                x = box.xmax - box.xmin
                y = box.ymax - box.ymin
                self.sign_size_b[box.Class] = x*y
    

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
            result= order_b[0] + '>' + order_b[1]
            self.b_compare[result] +=1

        elif b_count ==3:

            result= order_b[0] + '>' + order_b[1]
            self.b_compare[result] += 1

            result= order_b[1] + '>' + order_b[2]
            self.b_compare[result] += 1

            result= order_b[0] + '>' + order_b[2]
            self.b_compare[result] += 1


        # print(self.b_compare)
        
    def target_b_decision(self, maxClassA):
        print('maxA', maxClassA)

        target_b = -1
        B1B2 = self.b_compare['B1>B2'] > self.b_compare['B2>B1'] # True/False

        B2B3 = self.b_compare['B2>B3'] > self.b_compare['B3>B2'] # True/False

        B3B1 = self.b_compare['B3>B1'] > self.b_compare['B1>B3'] # True/False
        if B1B2:
            if B2B3:
                result = [1, 2, 3]
            else:
                if B3B1:
                    result = [3, 1, 2]
                else:
                    result = [1, 3, 2]
        else:
            if B2B3:
                if B3B1:
                    result = [2, 3, 1]
                else:
                    result = [2, 1, 3]
            else:
                result = [3, 2, 1]
        print('result', result)
        target_b = result.index(int(maxClassA[1]))
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


