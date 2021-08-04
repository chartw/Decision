import numpy as np
from numpy import argmax, array
from geometry_msgs.msg import Point32
from math import hypot
import csv

from master_node.msg import Path

from lib.planner_utils.cubic_spline_planner import Spline2D

class deliveryClass:
    def __init__(self):
        self.sign_count_a = {'A1':0, 'A2': 0, 'A3':0}
        self.sign_names = ['A1', 'A2', 'A3', 'B1', 'B2', 'B3']
        self.ratio_th = 1.5
        self.size_th = 2
        self.queue = []
        self.queue_size=10
        self.sign_size_b = {'B1':0, 'B2': 0, 'B3':0}
        self.delivery_path_a = []
        self.delivery_path_b = []


    def make_delivery_path(self):
        with open("./map/kcity_map/final.csv", mode="r") as csv_file:
            csv_reader = csv.reader(csv_file)
            for line in csv_reader:
                if line[6] == "delivery_a":
                    point = (float(line[0]), float(line[1]))
                    self.delivery_path_a.append(point)
                elif line[6] == "delivery_b":
                    point = (float(line[0]), float(line[1]))
                    self.delivery_path_b.append(point)

        return self.delivery_path_a, self.delivery_path_b

    def detect_signs(self, boxes):
        max_count = 0
        maxClassA = ''
        maxClassB = ''
        for box in boxes:
            if box.Class in self.sign_names[0:3]: #A1~A3
                self.sign_count_a[box.Class] += 1
                if max_count < self.sign_count_a[box.Class]:
                    max_count = self.sign_count_a[box.Class]
                    maxClassA = box.Class
             
            elif box.Class in self.sign_names[3:6]: #B1~B3
                x = box.xmax - box.xmin
                y = box.ymax - box.ymin
                self.sign_size_b[box.Class] = x*y

        sorted_signs_b = sorted(self.sign_size_b.items(), key=lambda x: x[1], reverse=True)
        order_b = []

        for i in range(3):
            order_b.append(sorted_signs_b[i][0])

        #초기화
        self.sign_size_b = {'B1':0, 'B2': 0, 'B3':0}

        return maxClassA, order_b

    def path_plan(self, path, sign_index):
        path=Path()

        x_list, y_list=[],[]
        x_list.append[path.x[0]]
        y_list.append[path.y[0]]
        x_list.append[path.x[sign_index]]
        y_list.append[path.y[sign_index]]

        csp=Spline2D(x_list, y_list)
        s=np.arange(0,csp.s[-1],0.1)
        rx, ry=[],[]
        for i_s in s:
            ix,iy=csp.calc_position(i_s)
            rx.append(ix)
            ry.append(iy)


        path.x=rx
        path.y=ry
        return path

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
        if distance < 0.5:
            return True

    def index_decision(self, order_b, sign_map, targetPoint):
        goal_order=order_b.index(targetPoint)

        return sign_map[goal_order].index



