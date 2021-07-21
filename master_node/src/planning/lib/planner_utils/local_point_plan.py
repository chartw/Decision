import numpy as np
from math import cos, sin, hypot,pi
from geometry_msgs.msg import Point32

class LPP:
    def point_plan(self, segments):
        point=Point32()
        longest = segments[0]
        for segment in segments:
            if segment.last_point.x < segment.first_point.x:
                temp = segment.last_point
                segment.last_point = segment.first_point
                segment.first_point = temp

            if hypot(longest.first_point.x-longest.last_point.x, longest.first_point.y-longest.last_point.y) < \
             hypot(segment.first_point.x-segment.last_point.x, segment.first_point.y-segment.last_point.y):
                longest = segment

        if longest.first_point.x < 1:
            point=Point32(1.5,0,0)
            return point
        d = 2
        rad = np.arctan2(longest.last_point.y - longest.first_point.y, longest.last_point.x - longest.first_point.x)

        if  abs(rad)<2:
            d=1.5
            if longest.first_point.y >0:
                rad += -pi/2
            else:
                rad += +pi/2
        point.x = longest.last_point.x + (d * cos(rad))
        point.y = longest.last_point.y + (d * sin(rad))
        print(point.x, point.y, rad)

        return point
