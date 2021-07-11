from math import degrees, atan2, sin, radians
from master_node.msg import Serial_Info()


class Avoidance:
    def __init__(self, control):
        self.point = control.planning_info.point
        self.temp_msg = Serial_Info()

    def pure_puresuit(self):
            tmp_th = degrees(atan2((self.point.y - 0), (self.point.x - 0)))
            tmp_th = tmp_th%360

            alpha =  0 - tmp_th

            if abs(alpha)>180:
                if (alpha < 0) :
                    alpha += 360
                else :
                    alpha -= 360

            alpha = max(alpha,  -90)
            alpha = min(alpha,  90)

            delta = alpha
  
            if abs(delta)>180:
                if (delta < 0) :
                    delta += 360
                else :
                    delta -= 360
            # print("delta", delta)
            if abs(delta)>30:
                if delta > 0:
                    return 27.7
                else :
                    return -27.7
            else:
                return delta

    def driving(self):
        self.temp_msg.steer = self.pure_pursuit()
        self.temp_msg.speed = 8
        self.temp_msg.brake = 0
        self.temp_msg.encoder = 0
        self.temp_msg.gear = 0
        self.temp_msg.emergency_stop = 0
        self.temp_msg.auto_manual = 1

        return self.temp_msg