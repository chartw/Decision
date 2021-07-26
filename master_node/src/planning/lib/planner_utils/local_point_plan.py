import numpy as np
from math import cos, sin, hypot, pi, radians, sqrt
from geometry_msgs.msg import Point32
from master_node.msg import Path

from lib.planner_utils.cubic_spline_planner import Spline2D

class LPP:
    def __init__(self,planner):
        self.point = Point32()
        self.local=planner.local
        self.global_path=planner.global_path
        self.base_index=0
        self.base=Point32()
        self.local_path=Path()
        self.target_index=0
        self.target_point_dict={}

    def start(self,planner):
        # self.target_point_dict={}
        self.local_path=Path()
        self.global_path=planner.global_path
        self.base_index=planner.veh_index
        self.base=Point32(self.global_path.x[self.base_index],self.global_path.y[self.base_index],0)

    def path_plan(self,obstacles):
        self.target_point_dict={}
        for obstacle in obstacles:
            # 장애물 절대좌표 변환

            min_dist=-1
            min_index = 0
            # base_index = avoidance가 시작된 시점의 차량의 위치와 가장 가까운 global path index
            # base_index 부터 장애물과 가장 가까운 index를 탐색한다.
            # min_dist < dist인 경우, 가장 가까운 index를 지나쳤다고 판단하여 break
            end_index=min(self.base_index+300, len(self.global_path.x)-1)
            for i in range(self.base_index,end_index):
                dist=hypot(self.global_path.x[i]-obstacle.x, self.global_path.y[i]-obstacle.y)
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
            rad=np.arctan2(obstacle.y-self.global_path.y[min_index], obstacle.x-self.global_path.x[min_index]) + pi
            point=Point32()
            point.x = self.global_path.x[min_index] + (r * cos(rad))
            point.y = self.global_path.y[min_index] + (r * sin(rad))

            self.target_point_dict[min_index]=point

        # target point를 key로 정렬 -> tuple로 이루어진 list[(index, point), (index, point) ... ]
        target_point_list=sorted(self.target_point_dict.items())
        if len(target_point_list) ==0:
            return self.local_path

        # 마지막 target point로부터 20 index 만큼 떨어진 점을 마지막으로 추가함
        # out of index 방지 위해 min 활용
        last_index=min(target_point_list[-1][0]+50, len(self.global_path.x)-1)
        target_point_list.append((last_index, Point32(self.global_path.x[last_index],self.global_path.y[last_index],0)))

        # cubic spline으로 경로 생성 local -> target_point_list[0][1] -> target_point_list[1][1]...
        x_list, y_list=[],[]
        for i in range(len(target_point_list)):
            x_list.append(target_point_list[i][1].x)
            y_list.append(target_point_list[i][1].y)

        csp=Spline2D(x_list, y_list)
        s=np.arange(0,csp.s[-1],0.1)
        rx, ry=[],[]
        for i_s in s:
            ix,iy=csp.calc_position(i_s)
            rx.append(ix)
            ry.append(iy)

        self.local_path.x=rx
        self.local_path.y=ry
        return self.local_path

# 경로가 짧기때문에 0번인덱스부터 계속 탐색해도 괜찮을것 같다.
    def point_plan(self, planner, lookahead):
        valid_idx_list = []
        min_idx=0
        min_dist=-1
        for i in range(len(self.local_path.x)):
            dis = hypot(self.local_path.x[i] - self.local.x, self.local_path.y[i] - self.local.y)
            if dis < min_dist or min_dist == -1:
                min_dist=dis
                min_idx=i
                
            if dis <= lookahead:
                valid_idx_list.append(i)
            if len(valid_idx_list) != 0 and dis > lookahead:
                break

        if valid_idx_list:
            self.target_index = valid_idx_list[len(valid_idx_list) - 1]
        else:
            self.target_index=min(min_idx+10,len(self.local_path.x)-1)
        # print(self.target_index)
        target_point=Point32(self.local_path.x[self.target_index],self.local_path.y[self.target_index],0)
        return target_point