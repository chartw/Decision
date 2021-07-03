import numpy as np
from math import cos, sin


class LPP:
    def __init__(self, planner):
        self.segments = planner.obstacles.segments

    def point_plan(self):
        closest = self.segments[0]
        for segment in self.segments:
            if segment.last_point.x < segment.first_point.x:
                temp = segment.last_point
                segment.last_point = segment.first_point
                segment.first_point = temp

            if self.calc_dis(closest.first_point.x, closest.first_point.y) > self.calc_dis(segment.first_point.x, segment.first_point.y):
                closest = segment

        if closest.first_point.x < 1:
            return 1, 0
        d = 1.5
        rad = np.arctan2(closest.last_point.y - closest.first_point.y, closest.last_point.x - closest.first_point.x)
        # print(rad)
        return closest.last_point.x + (d * cos(rad)), closest.last_point.y + (d * sin(rad))

    def calc_dis(self, nx, ny):
        distance = ((nx - 0 ** 2) + (ny - 0) ** 2) ** 0.5
        return distance
