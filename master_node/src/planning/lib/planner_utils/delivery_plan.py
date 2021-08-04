from geometry_msgs.msg import Point32
from lib.planner_utils.local_point_plan import LPP
from lib.planner_utils.mission_plan import MissionPlan

def find_stop_point(self, targetPoint):
    min = 100
    min_x, min_y = 0

    for i in range(slicedPath):
        distance = hypot(slicedPath[i].x, slicedPath[i].y, targetPoint.x, targetPoint.y)
        if min > distance:
            min = distance
            min_x = slicedPath[i].x
            min_y = slicedPath[i].y

    return min_x, min_y

def stop_decision(self, targetPointX, targetPointY):
    distance = hypot(self.local.x, self.local.y, targetPointX, targetPointY)
    if distance < 0.5:
        return True
    else return False

def index_decision(self, order_b, coordinate_b, targetIndex):
    for i in range(order_b):
        if order_b[i] == targetIndex:
            return coordinate_b[order_b[i]]
    
    return [[-1, -1]] # 이거맞나?