import numpy as np
from math import cos, sin
from geometry_msgs.msg import Point32


class LPP:
    def point_plan(self, segments):
        point=Point32()
        closest = segments[0]
        for segment in segments:
            if segment.last_point.x < segment.first_point.x:
                temp = segment.last_point
                segment.last_point = segment.first_point
                segment.first_point = temp

            if self.calc_dis(closest.first_point.x, closest.first_point.y) > self.calc_dis(segment.first_point.x, segment.first_point.y):
                closest = segment

        if closest.first_point.x < 1:
            point=Point32(1.5,0,0)
            return point
        d = 1.0
        rad = np.arctan2(closest.last_point.y - closest.first_point.y, closest.last_point.x - closest.first_point.x)
        print(rad)
        point.x= closest.last_point.x + (d * cos(rad))
        point.y= closest.last_point.y + (d * sin(rad))
        return point

    def calc_dis(self, nx, ny):
        distance = ((nx - 0 ** 2) + (ny - 0) ** 2) ** 0.5
        return distance
