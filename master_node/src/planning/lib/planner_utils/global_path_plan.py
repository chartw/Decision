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

from math import cos, hypot
from math import atan2
from math import sin
from math import degrees
from math import radians

from master_node.msg import Path
from geometry_msgs.msg import Point32


class Graph:  # graph
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


class node:  # node
    def __init__(self, name, x, y):
        self.name = name
        self.x = x
        self.y = y


class GPP:
    def __init__(self, planner):
        self.local = planner.local
        self.map=planner.mapname

        self.nodelist = {}
        if self.map=="songdo" or self.map=="kcity":
            self.goal_id = planner.goal_node
            self.node_set(self.map)
            self.lane_set(self.map)

        self.global_path=Path()
        self.target_index=0

    def astar_search(self, graph, mynode, start, end):  # A* search (graph)
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
                return "({0},{1})".format(self.name, self.f)

        def add_to_open(open, neighbor):
            for node in open:  # open에 있는 있는 것보다 f가 크면 추가 안해줌
                if neighbor == node and neighbor.f > node.f:
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

        while len(open) > 0:  # open이 empty될때 까지
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
                if neighbor in closed:
                    continue
                neighbor.g = current_node.g + graph.get(
                    current_node.name, neighbor.name
                )
                neighbor.h = heuristic(
                    mynode[current_node.name], mynode[neighbor.name]
                )
                neighbor.f = neighbor.g + neighbor.h
                if (
                    add_to_open(open, neighbor) == True
                ):  # open에 있는 있는 것보다 f가 크면 추가 안해줌
                    open.append(neighbor)
        return None  # 목적지 까지 경로가 없을때 아무것도 출력하지 않음

    def lane_set(self, mapname):
        # 만약 g나 h값을 코드상에 미리 넣어서 계산해준다면 모든 csv파일을 읽어올 필요는 없음.
        global files
        files = os.listdir("./map/" + mapname + "_lane/")

        # lane connect

        for file in files:
            tmp_name = file[0:4]

            globals()["Lane{}".format(tmp_name)] = {
                "id": tmp_name,
                "x": [],
                "y": [],
                "yaw": [],
                "k": [],
                "s": [],
                "g_cost": 0.0,
            }

            # csv파일위치
            csv_file = "./map/" + mapname + "_lane/" + file

            with open(csv_file, mode="r") as map_file:

                csv_reader = csv.reader(map_file)

                for line in csv_reader:
                    globals()["Lane{}".format(tmp_name)]["x"].append(
                        float(line[0])
                    )
                    globals()["Lane{}".format(tmp_name)]["y"].append(
                        float(line[1])
                    )
                    if math.degrees(float(line[2])) < 0:
                        deg_yaw = math.degrees(float(line[2])) + 360
                    else:
                        deg_yaw = math.degrees(float(line[2]))
                    if deg_yaw > 90 and deg_yaw <= 360:
                        deg_yaw -= 90
                    else:
                        deg_yaw += 270

                    globals()["Lane{}".format(tmp_name)]["yaw"].append(deg_yaw)
                    globals()["Lane{}".format(tmp_name)]["k"].append(
                        float(line[3])
                    )
                    globals()["Lane{}".format(tmp_name)]["s"].append(
                        float(line[4])
                    )
                globals()["Lane{}".format(tmp_name)]["g_cost"] = 0.1 * len(
                    globals()["Lane{}".format(tmp_name)]["x"]
                )
                # print('Lane{}'.format(tmp_name), globals()['Lane{}'.format(tmp_name)]['g_cost'])

            graph.connect(
                file[0:2],
                file[2:4],
                globals()["Lane{}".format(tmp_name)]["g_cost"],
            )
            map_file.close()

    def node_set(self, mapname):
        with open("./map/" + mapname + "_node.csv", mode="r") as csv_file:
            csv_reader = csv.DictReader(csv_file)
            for next_r in csv_reader:
                self.nodelist[next_r["nid"]] = node(
                    str(next_r["nid"]), float(next_r["nx"]), float(next_r["ny"])
                )

    def path_plan(self):
        if self.map!="songdo":
            self.make_global_map()
            return self.global_path
        
        else:
            start_id = self.select_start_node()
            goal_ids = self.goal_id.split("/")
            path = self.astar_search(graph, self.nodelist, start_id, goal_ids[0])
            if len(goal_ids) > 1:
                for i in range(len(goal_ids) - 1):
                    temppath = self.astar_search(
                        graph, self.nodelist, goal_ids[i], goal_ids[i + 1]
                    )
                    temppath.pop(0)
                    path += temppath
            for i in range(len(path) - 1):

                # print('Lane{}'.format(path[i]+path[i+1]))

                for j in range(
                    len(globals()["Lane{}".format(path[i] + path[i + 1])]["x"])
                ):
                    self.global_path.x.append(
                        globals()["Lane{}".format(path[i] + path[i + 1])]["x"][j]
                    )
                    self.global_path.y.append(
                        globals()["Lane{}".format(path[i] + path[i + 1])]["y"][j]
                    )
                    pyaw = (
                        globals()["Lane{}".format(path[i] + path[i + 1])]["yaw"][j]
                    )
                    
                    pyaw= (degrees(pyaw)+360) % 360
                    self.global_path.heading.append(pyaw)
                    self.global_path.k.append(
                        globals()["Lane{}".format(path[i] + path[i + 1])]["k"][j]
                    )

            return self.global_path
            # 가장 가까운 노드를 시작 노드로 설정

    def select_start_node(self):
        nodelist = self.nodelist

        min_dis = 99999
        min_idx = 10000
        temp_idx = 10000
        temp_dis = 9999

        for node in nodelist:
            temp_dis = hypot(self.local.x-nodelist[node].x, self.local.y-nodelist[node].y)
            if temp_dis < min_dis:
                min_dis = temp_dis
                min_idx = node
        return min_idx

    def point_plan(self, planner, lookahead):
        valid_idx_list = []
        idx = 0
        min_idx=0
        min_dist=-1
        start=max(0,planner.veh_index-100)
        end=min(len(self.global_path.x)-1,planner.veh_index+100)
        for i in range(start,end):
            dis = hypot(self.global_path.x[i] - self.local.x, self.global_path.y[i] - self.local.y)

            if dis < min_dist or min_dist == -1:
                min_dist=dis
                min_idx=i
            
        planner.veh_index=min_idx
        self.target_index=min_idx+40
        target_point=Point32(self.global_path.x[self.target_index],self.global_path.y[self.target_index],0)
        # print(self.global_path.heading[self.target_index])
        return self.target_index, target_point

    def make_global_map(self):
        with open("./map/kcity_map/" + self.map + ".csv", mode="r") as csv_file:
            csv_reader = csv.reader(csv_file)


            for line in csv_reader:
                self.global_path.x.append(float(line[0]))
                self.global_path.y.append(float(line[1]))

                deg_yaw=(degrees(float(line[2]))+360) % 360
                self.global_path.heading.append(deg_yaw)

                self.global_path.k.append(float(line[3]))
                self.global_path.s.append(float(line[4]))
                # self.global_path.env.append(line[5])
                # self.global_path.mission.append(line[6])


########################################밑에 다지우고 위에 주석풀면 원래코드

            # x = []
            # y = []
            # k = []
            # yaw = []
            # i = 0
            # for line in csv_reader:
            #     x.append(float(line[0]))
            #     y.append(float(line[1]))

            #     deg_yaw=(degrees(float(line[2]))+360) % 360
            #     yaw.append(deg_yaw)

            #     k.append(float(line[3]))
            #     # self.global_path.s.append(float(line[4]))
            #     self.global_path.env.append(line[5])
            #     self.global_path.mission.append(line[6])
            #     i += 1
            #     if i == 1219 :
            #         break;
            #     print(i)
            # i-=1
            # print("ㅁㅎ여ㅑ호")
            
            # for _ in range(1218):
            #     if i == 0 :
            #         break;
            #     self.global_path.x.append(x[i])
            #     self.global_path.y.append(y[i])

            #     self.global_path.k.append(k[i])
            #     self.global_path.heading.append(yaw[i])

            #     i-=1
                
