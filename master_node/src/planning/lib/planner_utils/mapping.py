from lib.planner_utils.move_avg_filter import MovAvgFilter
from master_node.msg import Local
from geometry_msgs.msg import Point32
from sensor_msgs.msg import PointCloud

from math import radians, cos, sin, hypot

class Mapping:
    map={}
    obstacle_cnt=0
    local=Local()
    def __init__ (self, planner):
        self.local=planner.local

    def mapping(self, planner,circles):
        theta=radians(planner.local.heading)
        for circle in circles:
            # 장애물 절대좌표 변환
            x=circle.center.x*cos(theta)+circle.center.y*-sin(theta) + planner.local.x
            y=circle.center.x*sin(theta)+circle.center.y*cos(theta) + planner.local.y
            id=-1
            for i, MAF in self.map.items():
                if hypot(x-MAF.prevAvg.x,y-MAF.prevAvg.y) < 2.5:
                    id=i
                    break

            if id==-1:
                new_obstacle=MovAvgFilter(10000, Point32(x,y,0))
                self.map[self.obstacle_cnt]=new_obstacle
                self.obstacle_cnt+=1
            else:
                self.map[id].movAvgFilter(Point32(x,y,0))
        

    def showObstacleMap(self):
        obstacle_map=PointCloud()
        for i,MAF in self.map.items():
            obstacle_map.points.append(MAF.prevAvg)

        return obstacle_map
            