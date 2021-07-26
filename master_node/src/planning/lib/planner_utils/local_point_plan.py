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

    def path_plan(self,target_map):
        target_point_list=sorted(target_map.items())

        if len(target_point_list) ==0:
            return self.local_path

        # 마지막 target point로부터 20 index 만큼 떨어진 점을 마지막으로 추가함
        # out of index 방지 위해 min 활용
        last_index=min(target_point_list[-1][0]+50, len(self.global_path.x)-1)

        # cubic spline으로 경로 생성 local -> target_point_list[0][1] -> target_point_list[1][1]...
        x_list, y_list=[],[]
        for i in range(len(target_point_list)):
            x_list.append(target_point_list[i][1].point.x)
            y_list.append(target_point_list[i][1].point.y)
        
        x_list.append(self.global_path.x[last_index])
        y_list.append(self.global_path.y[last_index])

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