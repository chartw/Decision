from lib.control_utils.general import General
from math import degrees, atan2, sin, radians,pi

class Parking:
    def __init__(self, control):
        self.generalClass = General(control)

        self.cur = control.local
        self.lookahead = 1
        self.WB = 1.04

    # def drivingParkingNode(self, parking_point):
    #     print('===========================================', parking_point)
    #     parking_point = 
    #     steer = self.pure_pursuit(parking_point)

    #     return steer

    # def pure_pursuit(self, point, control):
    #     # pure pursuit 계산되는 부분
    #     self.cur = control.local
    #     tmp_th = degrees(atan2((point.y - self.cur.y), (point.x - self.cur.x)))

    #     tmp_th = tmp_th % 360
    #     if control.planning_info.mode=="parking_backward":
    #         self.cur.heading=(180+self.cur.heading)%360
    #     alpha = self.cur.heading - tmp_th
    #     if abs(alpha) > 180:
    #         if alpha < 0:
    #             alpha += 360
    #         else:
    #             alpha -= 360

    #     alpha = max(alpha, -90)
    #     alpha = min(alpha, 90)

    #     delta = degrees(atan2(2 * self.WB * sin(radians(alpha)) / self.lookahead, 1))

    #     if abs(delta) > 180:
    #         if delta < 0:
    #             delta += 360
    #         else:
    #             delta -= 360

    #     else:
    #         return delta #steer

    def pure_pursuit(self, point, control):
        # pure pursuit 계산되는 부분
        self.cur = control.local
        tmp_th = atan2((point.y - self.cur.y), (point.x - self.cur.x))

        heading=radians(self.cur.heading)
        if control.planning_info.mode=="parking_backward":
            heading+=pi
        alpha = heading - tmp_th

        delta = degrees(atan2(2 * self.WB * sin(alpha) / self.lookahead, 1))


        return delta #steer

    def fpure_pursuit(self, point, control):

        tmp_th = degrees(atan2((point.y - control.local.y), (point.x - control.local.x)))

        tmp_th = tmp_th % 360

        alpha = control.local.heading - tmp_th
        if abs(alpha) > 180:
            if alpha < 0:
                alpha += 360
            else:
                alpha -= 360

        alpha = max(alpha, -90)
        alpha = min(alpha, 90)

        delta = degrees(atan2(2 * self.WB * sin(radians(alpha)) / self.lookahead, 1))

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
