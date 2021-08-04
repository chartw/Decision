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

def stop_decision(self, targetPoint):
    distance = hypot(self.local.x, self.local.y, targetPoint.x, targetPoint.y)
    if distance < 0.5:
        return true

def index_decision(self, order_b, coordinate_b, targetPoint):
    goal_index = ''
    for i in range(order_b):
        if order_b[i] == targetPoint:
            goal_index = order_b[i]
            break

    return coordinate_b[goal_index]