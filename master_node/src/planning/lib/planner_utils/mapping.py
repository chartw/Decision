import numpy as np
from lib.planner_utils.exp_mov_avg import ExpMovAvgFilter
from master_node.msg import Local
from geometry_msgs.msg import Point32
from sensor_msgs.msg import PointCloud

from math import radians, cos, sin, hypot, pi

class Obstacle:
    dist=0
    index=0
    point=Point32()
    rad=None
    EMA=ExpMovAvgFilter(Point32())
    def __init__(self, idx, dist, ema):
        self.index=idx
        self.dist=dist
        self.EMA=ema

class Mapping:
    obs_map = {} # 실제 장애물 좌표가 저장되는 dictionary
    obstacle_cnt = 0 # 현재 장애물의 개수
    local = Local() # 위치 정보

    def __init__(self, planner):
        self.local = planner.local

    def start(self):
        self.obs_map={}
        self.obstacle_cnt=0
        # 원래 init에서 주소를 copy해서 할려고햇는데, 뭔가 잘 안되서, 확실하게 하기위해 planner.local로 함
    def mapping(self, planner, circles):
        theta = radians(planner.local.heading)
        for circle in circles:
            # 현재 mapping 중인 장애물 : circle
            # 장애물 절대좌표 변환
            pos=Point32()
            pos.x = circle.center.x * cos(theta) + circle.center.y * -sin(theta) + planner.local.x
            pos.y = circle.center.x * sin(theta) + circle.center.y * cos(theta) + planner.local.y
            id = -1

            for i, obstacle in self.obs_map.items():
                if obstacle.EMA.tracking(pos):
                    id = i
                    break
            
            # 만약 딕셔너리에 없으면, 새로운 이동 평균 필터 클래스 선언후 삽입
            # circle의 절대좌표 (x, y)로 초기화
            if id == -1:
                min_dist=-1
                min_index = 0
                # base_index = avoidance가 시작된 시점의 차량의 위치와 가장 가까운 global path index
                # base_index 부터 장애물과 가장 가까운 index를 탐색한다.
                # min_dist < dist인 경우, 가장 가까운 index를 지나쳤다고 판단하여 break
                end_index=min(planner.veh_index+150, len(planner.global_path.x)-1)
                if end_index==len(planner.global_path.x)-1:
                    end_index=planner.veh_index+150 - (len(planner.global_path.x)-1)

                if end_index < planner.veh_index:
                    start_index=0
                else:
                    start_index=planner.veh_index

                for i in range(start_index,end_index):
                    dist=hypot(planner.global_path.x[i]-pos.x, planner.global_path.y[i]-pos.y)
                    if min_dist==-1 or min_dist > dist:
                        min_dist=dist
                        min_index = i

                self.obs_map[self.obstacle_cnt] = Obstacle(min_index,min_dist,ExpMovAvgFilter(pos))
                self.obstacle_cnt += 1

            # 있으면, 해당 key값의 이동평균 필터에 circle의 절대좌표 (x, y) 삽입
            else:
                self.obs_map[id].EMA.emaFilter(pos)
                obstacle=self.obs_map[id]
                emapos=obstacle.EMA.retAvg()

                min_dist=-1
                min_index=0
                start_index=max(0,obstacle.index-10)
                end_index=min(obstacle.index+10,len(planner.global_path.x)-1)

                for i in range(start_index,end_index):
                    dist=hypot(planner.global_path.x[i]-emapos.x, planner.global_path.y[i]-emapos.y)
                    if min_dist==-1 or min_dist > dist:
                        min_dist=dist
                        min_index = i

                self.obs_map[id].index=min_index
                self.obs_map[id].dist=min_dist
                


    def make_target_map(self,planner):
        target_map={}
        for i, obstacle in self.obs_map.items():
            dist=obstacle.dist
            index=obstacle.index
            emapos=obstacle.EMA.retAvg()
            if dist > 3:
                continue

            # dist -> 0 : 2 // dist -> inf : 0 식을 이용
            # 경로와 가까울때는 최대 2정도의 거리만큼 떨어져서 주행
            # 경로와 멀때는 거의 0에 가까운 거리만큼 떨어져서 주행
            dist=min(dist, 2)
            # r=sqrt(-9/10*min_dist + 9)
            # r=sqrt(-2/5*min_dist + 4)
            # r=sqrt(-3/4*min_dist + 9/4)
            # r=-3*min_dist/4 + 3/2
            r=-dist+2
            
            # r=1/(min_dist+1/3)
            # 경로의 반대쪽에 point를 찍음
            rad=np.arctan2(emapos.y-planner.global_path.y[index], emapos.x-planner.global_path.x[index]) + pi
            if obstacle.rad!=None:
                if abs(rad -obstacle.rad) > pi/2:
                    rad=obstacle.rad
            else:
                obstacle.rad=rad
            point=Point32()
            point.x = planner.global_path.x[index] + (r * cos(rad))
            point.y = planner.global_path.y[index] + (r * sin(rad))
            target_map[index]=point

        return target_map

        # target point를 key로 정렬 -> tuple로 이루어진 list[(index, point), (index, point) ... ]

    # 현재 map에 저장되어있는 모든 point를 PointCloud형식으로 바꿔서 리턴하는 함수
    # 이걸 바로 rviz로 쏘고있음
    def showObstacleMap(self):
        obstacle_map = PointCloud()
        for i, obstacle in self.obs_map.items():
            obstacle_map.points.append(obstacle.EMA.retAvg())

        return obstacle_map
