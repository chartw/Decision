import numpy as np
from lib.planner_utils.exp_mov_avg import ExpMovAvgFilter
from master_node.msg import Local
from geometry_msgs.msg import Point32
from sensor_msgs.msg import PointCloud

from math import radians, cos, sin, hypot, pi, degrees


class Obstacle:
    dist = 0
    index = 0
    point = Point32()
    rad = None
    EMA = ExpMovAvgFilter(Point32())
    update = False
    cnt = 0

    def __init__(self, idx, dist, ema):
        self.index = idx
        self.dist = dist
        self.EMA = ema
        self.update = True


class Mapping:
    obs_map = {}  # 실제 장애물 좌표가 저장되는 dictionary
    a_sign_map = {}
    b_sign_map = {}
    obstacle_cnt = 0  # 현재 장애물의 개수
    local = Local()  # 위치 정보
    a_cnt = 0
    b_cnt = 0

    def initialize(self):
        self.obs_map = {}  # 실제 장애물 좌표가 저장되는 dictionary
        self.a_sign_map = {}
        self.b_sign_map = {}
        self.obstacle_cnt = 0  # 현재 장애물의 개수
        self.local = Local()  # 위치 정보
        self.a_cnt = 0
        self.b_cnt = 0


    def mapping(self, planner, circles, local):
        self.local = local
        theta = radians(self.local.heading)
        # print(self.local.heading)
        for circle in circles:
            # 현재 mapping 중인 장애물 : circle
            # 장애물 절대좌표 변환
            pos = Point32()
            pos.x = circle.center.x * cos(theta) + circle.center.y * -sin(theta) + self.local.x
            pos.y = circle.center.x * sin(theta) + circle.center.y * cos(theta) + self.local.y
            id = -1

            for i, obstacle in self.obs_map.items():
                if obstacle.EMA.tracking(pos):
                    id = i
                    break

            # 만약 딕셔너리에 없으면, 새로운 이동 평균 필터 클래스 선언후 삽입
            # circle의 절대좌표 (x, y)로 초기화
            if id == -1:

                min_dist = -1
                min_index = 0
                # base_index = avoidance가 시작된 시점의 차량의 위치와 가장 가까운 global path index
                # base_index 부터 장애물과 가장 가까운 index를 탐색한다.
                # min_dist < dist인 경우, 가장 가까운 index를 지나쳤다고 판단하여 break
                end_index = min(planner.veh_index + 150, len(planner.global_path.x) - 1)
                if end_index == len(planner.global_path.x) - 1:
                    end_index = planner.veh_index + 150 - (len(planner.global_path.x) - 1)

                if end_index < planner.veh_index:
                    start_index = 0
                else:
                    start_index = planner.veh_index
                for i in range(start_index, end_index):

                    dist = hypot(planner.global_path.x[i] - pos.x, planner.global_path.y[i] - pos.y)
                    if min_dist == -1 or min_dist > dist:
                        min_dist = dist
                        min_index = i

                if planner.global_path.mission[planner.veh_index] == "big":

                    # rad = np.arctan2(pos.y - planner.global_path.y[min_index], pos.x - planner.global_path.x[min_index])
                    # rad=degrees(rad) % 360
                    crossP = planner.global_path.x[min_index] * pos.y - planner.global_path.y[min_index] * pos.x
                    # print(min_dist, crossP)
                    if min_dist < 0.5:
                        pass

                    elif crossP < 0 and min_dist < 4:
                        pass

                    else:
                        continue

                    if min_index in self.obs_map:
                        continue

                    L = 3

                    for index in range(min_index - 10, int(min_index + L * 10) - 10):
                        pos = Point32(planner.global_path.x[index], planner.global_path.y[index], 0)
                        self.obs_map[index] = Obstacle(index, min_dist, ExpMovAvgFilter(pos))
                        self.obstacle_cnt += 1
                else:
                    self.obs_map[min_index] = Obstacle(min_index, min_dist, ExpMovAvgFilter(pos))
                    self.obstacle_cnt += 1

            # 있으면, 해당 key값 이동평균 필터에 circle의 절대좌표 (x, y) 삽입
            else:
                if planner.global_path.mission[planner.veh_index] == "big":
                    continue
                self.obs_map[id].EMA.emaFilter(pos)
                obstacle = self.obs_map[id]
                emapos = obstacle.EMA.retAvg()

                min_dist = -1
                min_index = 0
                start_index = max(0, obstacle.index - 10)
                end_index = min(obstacle.index + 10, len(planner.global_path.x) - 1)

                for i in range(start_index, end_index):
                    dist = hypot(planner.global_path.x[i] - emapos.x, planner.global_path.y[i] - emapos.y)
                    if min_dist == -1 or min_dist > dist:
                        min_dist = dist
                        min_index = i

                self.obs_map[id].index = min_index
                self.obs_map[id].dist = min_dist
                self.obs_map[id].update = True

            # 5 프레임에 동안 안나오면, 삭제
            # 단, 회피주행중일 경우 제외
        if planner.planning_msg.mode == "small" or planner.planning_msg.mode == "big":
            return

        for id in list(self.obs_map.keys()):
            if self.obs_map[id].update:
                self.obs_map[id].update = False
                self.obs_map[id].cnt = 0
            elif self.obs_map[id].cnt > 5:
                del self.obs_map[id]
            else:
                self.obs_map[id].cnt += 1

    def make_target_map(self, planner):
        target_map = {}
        if planner.planning_msg.mode == "big":
            for i, obstacle in self.obs_map.items():
                dist = obstacle.dist
                index = obstacle.index
                emapos = obstacle.EMA.retAvg()

                std_point = Point32(planner.global_path.x[index], planner.global_path.y[index], 0)
                if dist < 1:
                    rad = radians(planner.global_path.heading[index]) - pi / 2
                    r = 2
                else:
                    rad = radians(planner.global_path.heading[index]) + pi / 2
                    r = 0

                point = Point32()
                point.x = std_point.x + (r * cos(rad))
                point.y = std_point.y + (r * sin(rad))
                target_map[index] = point

        elif planner.planning_msg.mode == "small":
            for i, obstacle in self.obs_map.items():
                dist = obstacle.dist
                index = obstacle.index
                emapos = obstacle.EMA.retAvg()

                std_point = Point32(planner.global_path.x[index], planner.global_path.y[index], 0)
                if dist > 1.75:
                    continue

                # 경로와 가까울때는 최대 2정도의 거리만큼 떨어져서 주행
                # 경로와 멀때는 거의 0에 가까운 거리만큼 떨어져서 주행

                rad = np.arctan2(emapos.y - std_point.y, emapos.x - std_point.x)

                if rad - radians(planner.global_path.heading[index]) > 0 and dist < 0.5:
                    r = dist + 1.25
                else:
                    rad += pi
                    r = max(0, -dist + 1.25)

                if obstacle.rad != None:
                    if abs(rad - obstacle.rad) > pi / 2:
                        rad = obstacle.rad
                else:
                    obstacle.rad = rad
                point = Point32()
                point.x = std_point.x + (r * cos(rad))
                point.y = std_point.y + (r * sin(rad))
                target_map[index] = point
        return target_map

        # target point를 key로 정렬 -> tuple로 이루어진 list[(index, point), (index, point) ... ]

    ## 거리 조건 집어넣어서 벽 필터링 하자
    def a_sign_mapping(self, planner, path, circles):

        theta = radians(self.local.heading)
        for circle in circles:
            # 현재 mapping 중인 장애물 : circle
            # 장애물 절대좌표 변환
            pos = Point32()
            pos.x = circle.center.x * cos(theta) + circle.center.y * -sin(theta) + self.local.x
            pos.y = circle.center.x * sin(theta) + circle.center.y * cos(theta) + self.local.y
            id = -1

            for i, obstacle in self.a_sign_map.items():
                if obstacle.EMA.tracking(pos):
                    id = i
                    break

            # 만약 딕셔너리에 없으면, 새로운 이동 평균 필터 클래스 선언후 삽입
            # circle의 절대좌표 (x, y)로 초기화
            if id == -1:

                min_dist = -1
                min_index = 0

                for i in range(len(path.x)):
                    dist = hypot(path.x[i] - pos.x, path.y[i] - pos.y)
                    if min_dist == -1 or min_dist > dist:
                        min_dist = dist
                        min_index = i

                if min_dist > 3:
                    continue

                self.a_sign_map[self.a_cnt] = Obstacle(min_index, min_dist, ExpMovAvgFilter(pos))
                self.a_cnt += 1

            # 있으면, 해당 key값 이동평균 필터에 circle의 절대좌표 (x, y) 삽입
            else:
                self.a_sign_map[id].EMA.emaFilter(pos)
                obstacle = self.a_sign_map[id]
                emapos = obstacle.EMA.retAvg()

                min_dist = -1
                min_index = 0
                start_index = max(0, obstacle.index - 10)
                end_index = min(obstacle.index + 10, len(path.x) - 1)

                for i in range(start_index, end_index):
                    dist = hypot(path.x[i] - emapos.x, path.y[i] - emapos.y)
                    if min_dist == -1 or min_dist > dist:
                        min_dist = dist
                        min_index = i

                self.a_sign_map[id].index = min_index
                self.a_sign_map[id].dist = min_dist

        return self.a_sign_map

    def b_sign_mapping(self, planner, path, circles):
        theta = radians(self.local.heading)
        for circle in circles:
            # 현재 mapping 중인 장애물 : circle
            # 장애물 절대좌표 변환
            pos = Point32()
            pos.x = circle.center.x * cos(theta) + circle.center.y * -sin(theta) + self.local.x
            pos.y = circle.center.x * sin(theta) + circle.center.y * cos(theta) + self.local.y
            id = -1

            for i, obstacle in self.b_sign_map.items():
                if obstacle.EMA.tracking(pos):
                    id = i
                    break

            # 만약 딕셔너리에 없으면, 새로운 이동 평균 필터 클래스 선언후 삽입
            # circle의 절대좌표 (x, y)로 초기화
            if id == -1:

                min_dist = -1
                min_index = 0

                for i in range(len(path.x)):
                    dist = hypot(path.x[i] - pos.x, path.y[i] - pos.y)
                    if min_dist == -1 or min_dist > dist:
                        min_dist = dist
                        min_index = i
                if min_dist > 3:
                    continue

                self.b_sign_map[self.b_cnt] = Obstacle(min_index, min_dist, ExpMovAvgFilter(pos))
                self.b_cnt += 1

            # 있으면, 해당 key값 이동평균 필터에 circle의 절대좌표 (x, y) 삽입
            else:
                self.b_sign_map[id].EMA.emaFilter(pos)
                obstacle = self.b_sign_map[id]
                emapos = obstacle.EMA.retAvg()

                min_dist = -1
                min_index = 0
                start_index = max(0, obstacle.index - 10)
                end_index = min(obstacle.index + 10, len(path.x) - 1)

                for i in range(start_index, end_index):
                    dist = hypot(path.x[i] - emapos.x, path.y[i] - emapos.y)
                    if min_dist == -1 or min_dist > dist:
                        min_dist = dist
                        min_index = i

                self.b_sign_map[id].index = min_index
                self.b_sign_map[id].dist = min_dist

        return self.b_sign_map

    # 현재 map에 저장되어있는 모든 point를 PointCloud형식으로 바꿔서 리턴하는 함수
    # 이걸 바로 rviz로 쏘고있음
    def showObstacleMap(self):
        obstacle_map = PointCloud()
        for i, obstacle in list(self.obs_map.items()):
            obstacle_map.points.append(obstacle.EMA.retAvg())

        return obstacle_map
