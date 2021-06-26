# -*- coding:utf-8 -*-
# A* search (graph)
# read lanelet 
# lane_set 
# node_set 
# connect_path 
# by 전민규,차태웅


import os
import csv
import math

from math import cos
from math import atan2
from math import sin
from math import degrees
from math import radians

import matplotlib.pyplot as plt

tx, ty, tyaw = [], [], []
nodelist = {}

class Graph: #graph
    def __init__(self, graph_dict=None, directed=True):
        self.graph_dict = graph_dict or {}
        self.directed = directed
        if not directed:
            self.make_undirected()

    def make_undirected(self):
        for a in list(self.graph_dict.keys()):
            for (b, dist) in self.graph_dict[a].items():
                self.graph_dict.setdefault(b, {})[a] = dist

    def connect(self, A, B, distance):
        self.graph_dict.setdefault(A, {})[B] = distance
        # if not self.directed: # 이거는 단방향 실도로에서는 주석 ㄱㄱ 
        #     self.graph_dict.setdefault(B, {})[A] = distance

    def get(self, a, b=None):
        links = self.graph_dict.setdefault(a, {})
        if b is None:
            return links
        else:
            return links.get(b)

graph = Graph()

class node:  #node 
    def __init__(self, name, x, y):
        self.name = name
        self.x = x
        self.y = y

def astar_search(graph, mynode, start, end): #A* search (graph)
    class Node:
        def __init__(self, name, parent):
            self.name = name
            self.parent = parent
            self.g = 0
            self.h = 0
            self.f = 0

        # 연산자 오버로딩
        def __eq__(self, other):  # ==
            return self.name == other.name

        def __lt__(self, other):  # >,<
            return self.f < other.f

        def __repr__(self):  # print format
            return ('({0},{1})'.format(self.name, self.f))

    def add_to_open(open, neighbor):
        for node in open:  # open에 있는 있는 것보다 f가 크면 추가 안해줌
            if (neighbor == node and neighbor.f > node.f):
                return False
        return True

    def heuristic(now, goal):  # heuristic을 계산하는 함수
        dx = abs(now.x - goal.x)
        dy = abs(now.y - goal.y)
        return (dx * dx + dy * dy) ** 0.5

    open = []
    closed = []
    start_node = Node(start, None)
    goal_node = Node(end, None)
    open.append(start_node)

    while len(open) > 0: #open이 empty될때 까지
        # open에 저장된 Node중 f가 가장 낮은순서로 정렬후 현재 노드로 저장
        open.sort()
        current_node = open.pop(0)
        # 현재노드 closed에 추가
        closed.append(current_node)

        # 목적지 도착했으면 출력
        if current_node == goal_node:
            path = []
            while current_node != start_node:
                path.append(current_node.name)
                current_node = current_node.parent
            path.append(start_node.name)
            # Return
            return path[::-1]

        neighbors = graph.get(current_node.name)
        for key, value in neighbors.items():
            neighbor = Node(key, current_node)
            if (neighbor in closed):
                continue
            neighbor.g = current_node.g + graph.get(current_node.name, neighbor.name)
            neighbor.h = heuristic(mynode[current_node.name], mynode[neighbor.name])
            neighbor.f = neighbor.g + neighbor.h
            if (add_to_open(open, neighbor) == True): # open에 있는 있는 것보다 f가 크면 추가 안해줌
                open.append(neighbor)
    return None #목적지 까지 경로가 없을때 아무것도 출력하지 않음



def lane_set():

    #만약 g나 h값을 코드상에 미리 넣어서 계산해준다면 모든 csv파일을 읽어올 필요는 없음.
    global files 
    files = os.listdir("./map1/songdo_lane/")

    #lane connect

    
    
    for file in files:
        tmp_name = file[0:4]
    
        globals()['Lane{}'.format(tmp_name)]  = {'id': tmp_name, 'x':[], 'y':[],'yaw':[], 'k':[],'s':[], 'g_cost': 0.0}
       
       #csv파일위치
        csv_file = './map1/songdo_lane/' + file


        with open(csv_file, mode='r') as map_file:
           
            csv_reader = csv.reader(map_file)

           
            for line in csv_reader:
                globals()['Lane{}'.format(tmp_name)]['x'].append(float(line[0]))
                globals()['Lane{}'.format(tmp_name)]['y'].append(float(line[1]))
                if math.degrees(float(line[2])) < 0:
                    deg_yaw = math.degrees(float(line[2])) + 360
                else:
                    deg_yaw = math.degrees(float(line[2]))      
                if deg_yaw > 90 and deg_yaw <=360:
                    deg_yaw -= 90
                else:
                    deg_yaw+= 270 

                globals()['Lane{}'.format(tmp_name)]['yaw'].append(deg_yaw)
                globals()['Lane{}'.format(tmp_name)]['k'].append(float(line[3]))
                globals()['Lane{}'.format(tmp_name)]['s'].append(float(line[4]))
            globals()['Lane{}'.format(tmp_name)]['g_cost'] = 0.1*len(globals()['Lane{}'.format(tmp_name)]['x'])
            # print('Lane{}'.format(tmp_name), globals()['Lane{}'.format(tmp_name)]['g_cost'])       
               
        graph.connect(file[0:2], file[2:4],globals()['Lane{}'.format(tmp_name)]['g_cost'])
        map_file.close()

def node_set():
    with open('./map1/songdo_node.csv', mode='r') as csv_file:
        csv_reader = csv.DictReader(csv_file)
        for next_r in csv_reader:
            nodelist[next_r['nid']] = node(str(next_r['nid']),float(next_r['nx']),float(next_r['ny']))
    return nodelist

def path_connect(sid, gid):
    node_set()
    lane_set()
    path = astar_search(graph, nodelist, sid, gid)
    print(path)
    for i in range(len(path)-1):

        # print('Lane{}'.format(path[i]+path[i+1]))

        for j in range(len(globals()['Lane{}'.format(path[i]+path[i+1])]['x'])):
            tx.append(globals()['Lane{}'.format(path[i]+path[i+1])]['x'][j])
            ty.append(globals()['Lane{}'.format(path[i]+path[i+1])]['y'][j])
            pyaw = globals()['Lane{}'.format(path[i]+path[i+1])]['yaw'][j] + 90
            if pyaw >= 360:
                pyaw -=360
            tyaw.append(pyaw)

    return tx, ty, tyaw

def angle(cur_x,cur_y,cur_yaw,ta_x,ta_y):
    #tmp_th는 imu좌표계
    #atan2는 범위가 -파이~파이
    tmp_th = degrees(atan2((ta_y -cur_y),(ta_x-cur_x))) #degrees 파이를 각도로 변환하는 함수

    tmp_th = tmp_th%360

    #imu좌표에서는 90도 돌아가므로 90도 변환을 해줌

    if(tmp_th>90) and (tmp_th<=360):
         tmp_th -= 90
    else:
         tmp_th += 270


    #cur_yaw값은 각도로 주어졌다고 가정
    alpha =  cur_yaw - tmp_th #alpha

    if abs(alpha)>180: #abs는 절대값함수, alpha 범위
       if (alpha < 0 ):
           alpha += 360
       else :
           alpha -= 360


    alpha = max(alpha, -90)
    alpha = min(alpha,90)
    #+-90도 넘어가면 90도로 설정

    #델타(조향각) 계산
    #차길이 0.1  Ld 0.1*k =0.2
    delta = degrees(atan2(2*0.1*sin(radians(alpha))/0.2,1))
    if abs(delta)>180:
        if(delta<0):
            delta += 360
        else :
            delta -= 360

    #조향값의 크기가 커짐에 따라 줄수있는 방향의 최대최소값 제한은 생략
    #정방향 1550을 기준
    delta = 0 + (350/30)*delta
    return int(delta)
            


# def main():
#     node_set()
#     lane_set()
#     px, py = path_connect('50', '11')

#     plt.plot(px, py, ".r")

#     plt.show()
#     # for i in range(len(tx)):
#     #     steering=angle(cx, cy, cyaw, tx[i], ty[i]) # cx, cy, cyaw : GPS, IMU에서 받은 값


if __name__ == "__main__": main()