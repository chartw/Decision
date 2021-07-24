import numpy as np
from math import cos, sin, hypot, pi
from geometry_msgs.msg import Point32


class LPP:
    def __init__(self):
        self.point = Point32()

    def point_plan(self, circles):

        closest = circles[0]

        for circle in circles:
            if hypot(closest.center.x, closest.center.y) > hypot(circle.center.x, circle.center.y):
                closest = circle

        d = 2
        rad = np.arctan2(closest.center.y, closest.center.x)
        if closest.center.y > 0:
            rad -= pi / 2
        else:
            rad += pi / 2

        self.point.x = closest.center.x + (d * cos(rad))
        self.point.y = closest.center.y + (d * sin(rad))
        # print(self.point.x, self.point.y, rad)

        return self.point
