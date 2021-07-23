import numpy as np
from math import cos, sin, hypot,pi
from geometry_msgs.msg import Point32

class LPP:
    def __init__(self):
        self.point=Point32()

    def point_plan(self, circles):
        closest = None
        for circle in circles:
            if circle.center.x>-0.5:
                closest=circle
                break

        if closest == None:
            return self.point

        for circle in circles:
            if hypot(closest.center.x, closest.center.y) > hypot(circle.center.x, circle.center.y)and circle.center.x>-0.5:
                closest = circle



        # if closest.first_point.x < 1:
        #     point=Point32(1.5,0,0)
        #     return point
        d = 2
        rad = np.arctan2(closest.center.y, closest.center.x)
        if closest.center.y>0:
            rad-=pi/2
        else:
            rad+=pi/2

        self.point.x = closest.center.x + (d * cos(rad))
        self.point.y = closest.center.y + (d * sin(rad))
        # print(self.point.x, self.point.y, rad)

        return self.point
