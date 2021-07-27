from math import degrees, atan2, sin, radians
from master_node.msg import Serial_Info


class Avoidance:
    def __init__(self, control):
        self.local=control.local
        self.WB = 1.04


    def pure_pursuit(self, point):

        tmp_th = degrees(atan2((point.y - self.local.y), (point.x - self.local.x)))

        tmp_th = tmp_th % 360

        alpha = self.local.heading - tmp_th
        if abs(alpha) > 180:
            if alpha < 0:
                alpha += 360
            else:
                alpha -= 360

        alpha = max(alpha, -90)
        alpha = min(alpha, 90)

        delta = alpha

        if abs(delta) > 180:
            if delta < 0:
                delta += 360
            else:
                delta -= 360

        if abs(delta) >= 27.7:
            if delta > 0:
                return 27.7
            else:
                return -27.7
        else:

            return delta

    def driving(self, point):
        temp_msg=Serial_Info()
        temp_msg.steer = self.pure_pursuit(point)
        temp_msg.speed = 8
        temp_msg.brake = 0
        temp_msg.encoder = 0
        temp_msg.gear = 0
        temp_msg.emergency_stop = 0
        temp_msg.auto_manual = 1

        return temp_msg