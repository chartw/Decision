from lib.planner_utils.move_avg_filter import MovAvgFilter
from master_node.msg import Local
from geometry_msgs.msg import Point32
from sensor_msgs.msg import PointCloud

from math import radians, cos, sin, hypot


class Mapping:
    
    map = {} # 실제 장애물 좌표가 저장되는 dictionary
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
            for i, MAF in self.map.items():
                if hypot(x - MAF.prevAvg.x, y - MAF.prevAvg.y) < 2.5:
                    id = i
                    break
            
            # 만약 딕셔너리에 없으면, 새로운 이동 평균 필터 클래스 선언후 삽입
            # circle의 절대좌표 (x, y)로 초기화
            if id == -1:
                new_obstacle = MovAvgFilter(100, Point32(x, y, 0))
                self.map[self.obstacle_cnt] = new_obstacle
                self.obstacle_cnt += 1

            # 있으면, 해당 key값의 이동평균 필터에 circle의 절대좌표 (x, y) 삽입
            else:
                self.map[id].movAvgFilter(Point32(x, y, 0))

    # 현재 map에 저장되어있는 모든 point를 PointCloud형식으로 바꿔서 리턴하는 함수
    # 이걸 바로 rviz로 쏘고있음
    def showObstacleMap(self):
        obstacle_map = PointCloud()
        for i, MAF in self.map.items():
            obstacle_map.points.append(MAF.prevAvg)

        return obstacle_map
