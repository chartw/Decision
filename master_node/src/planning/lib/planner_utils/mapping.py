import numpy as np
from lib.planner_utils.exp_mov_avg import ExpMovAvgFilter
from master_node.msg import Local
from geometry_msgs.msg import Point32
from sensor_msgs.msg import PointCloud

from math import radians, cos, sin, hypot, pi

class Target:
    min_dist=0
    point=Point32()
    pos=Point32()
    
    def __init__(self, dist, p, pos):
        self.min_dist=dist
        self.point=p
        self.pos=pos


class Mapping:
    
    obs_map = {} # 실제 장애물 좌표가 저장되는 dictionary
    target_map={}
    obstacle_cnt = 0 # 현재 장애물의 개수
    local = Local() # 위치 정보

    def __init__(self, planner):
        self.local = planner.local

        # 원래 init에서 주소를 copy해서 할려고햇는데, 뭔가 잘 안되서, 확실하게 하기위해 planner.local로 함
    def mapping(self, planner, circles):
        theta = radians(planner.local.heading)
        for circle in circles:
            # 현재 mapping 중인 장애물 : circle
            # 장애물 절대좌표 변환
            x = circle.center.x * cos(theta) + circle.center.y * -sin(theta) + planner.local.x
            y = circle.center.x * sin(theta) + circle.center.y * cos(theta) + planner.local.y
            id = -1
            # 딕셔너리 탐색하며 circle과 거리가 2.5 이내인 점을 찾음

            for i, EMA in self.obs_map.items():
                if EMA.tracking(Point32(x,y,0)):
                    id = i
                    break
            
            # 만약 딕셔너리에 없으면, 새로운 이동 평균 필터 클래스 선언후 삽입
            # circle의 절대좌표 (x, y)로 초기화
            if id == -1:
                self.obs_map[self.obstacle_cnt] = ExpMovAvgFilter(Point32(x, y, 0))
                self.obstacle_cnt += 1

            # 있으면, 해당 key값의 이동평균 필터에 circle의 절대좌표 (x, y) 삽입
            else:
                self.obs_map[id].emaFilter(Point32(x, y, 0))

    def gpath_min_check(self,planner):
        t_map={}
        obstacles=self.showObstacleMap().points
        for obstacle in obstacles:
            # 장애물 절대좌표 변환
            min_dist=-1
            min_index = 0
            # base_index = avoidance가 시작된 시점의 차량의 위치와 가장 가까운 global path index
            # base_index 부터 장애물과 가장 가까운 index를 탐색한다.
            # min_dist < dist인 경우, 가장 가까운 index를 지나쳤다고 판단하여 break
            end_index=min(planner.veh_index+300, len(planner.global_path.x)-1)
            for i in range(planner.veh_index,end_index):
                dist=hypot(planner.global_path.x[i]-obstacle.x, planner.global_path.y[i]-obstacle.y)
                if min_dist==-1 or min_dist > dist:
                    min_dist=dist
                    min_index = i

            if min_dist > 3:
                continue                

            # dist -> 0 : 2 // dist -> inf : 0 식을 이용
            # 경로와 가까울때는 최대 2정도의 거리만큼 떨어져서 주행
            # 경로와 멀때는 거의 0에 가까운 거리만큼 떨어져서 주행
            min_dist=min(min_dist, 2)
            # r=sqrt(-9/10*min_dist + 9)
            # r=sqrt(-2/5*min_dist + 4)
            # r=sqrt(-3/4*min_dist + 9/4)
            # r=-3*min_dist/4 + 3/2
            r=-min_dist+2
            
            # r=1/(min_dist+1/3)
            # 경로의 반대쪽에 point를 찍음
            rad=np.arctan2(obstacle.y-planner.global_path.y[min_index], obstacle.x-planner.global_path.x[min_index]) + pi
            point=Point32()
            point.x = planner.global_path.x[min_index] + (r * cos(rad))
            point.y = planner.global_path.y[min_index] + (r * sin(rad))
            t=Target(min_dist, point, obstacle)
            t_map[min_index]=t

        self.target_map=t_map
        return self.target_map

        # target point를 key로 정렬 -> tuple로 이루어진 list[(index, point), (index, point) ... ]

    # 현재 map에 저장되어있는 모든 point를 PointCloud형식으로 바꿔서 리턴하는 함수
    # 이걸 바로 rviz로 쏘고있음
    def showObstacleMap(self):
        obstacle_map = PointCloud()
        for i, EMA in self.obs_map.items():
            obstacle_map.points.append(EMA.retAvg())

        return obstacle_map
