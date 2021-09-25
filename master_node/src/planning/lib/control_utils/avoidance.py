import rospy
from math import degrees, atan2, sin, radians, hypot, pi
from master_node.msg import Serial_Info
from geometry_msgs.msg import Point32
from sensor_msgs.msg import PointCloud



class Avoidance:
    def __init__(self, control):
        self.local=control.local
        self.WB = 1.04
        self.lookahead=3
        self.target_pub = rospy.Publisher("/target", PointCloud, queue_size=1)
        self.target_point=Point32()


    def point_plan(self, local_path):
        max_index=len(local_path.x)-1
        min_idx=0
        min_dist=-1
        for i in range(max_index+1):
            dis = hypot(local_path.x[i] - self.local.x, local_path.y[i] - self.local.y)
            if dis < min_dist or min_dist == -1:
                min_dist=dis
                min_idx=i
        if min_dist > self.lookahead:
            self.target_index=min_idx
        else:
            self.target_index=int(min(min_idx+(self.lookahead-min_dist)*10,max_index))
        # print(self.target_index)
        self.target_point=Point32(local_path.x[self.target_index],local_path.y[self.target_index],0)
        return self.target_point

    def pure_pursuit(self,control):

        point=self.point_plan(control.local_path)

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


    def parking_pure_pursuit(self,control):

        point=self.point_plan(control.local_path)

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

        delta = degrees(atan2(2 * self.WB * sin(radians(alpha)) / 2, 1))

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

    def bpure_pursuit(self, control):
        # pure pursuit 계산되는 부분

        point=self.point_plan(control.local_path)

        self.cur = control.local
        tmp_th = atan2((point.y - self.cur.y), (point.x - self.cur.x))

        heading=radians(self.cur.heading)
        # if control.planning_info.mode=="parking_backward":
        #     heading+=pi
        alpha = heading - tmp_th

        delta = degrees(atan2(2 * self.WB * sin(alpha) / 2, 1))

    

        return delta #steer

    def driving(self, control):
        temp_msg=Serial_Info()
        if control.planning_info.mode == "parking_backward":
            temp_msg.steer = self.bpure_pursuit(control)
            temp_msg.gear = 2
        elif control.planning_info.mode == "parking_start":
            temp_msg.steer = self.parking_pure_pursuit(control)
            temp_msg.gear = 0
        else:
            temp_msg.steer = self.pure_pursuit(control)
            temp_msg.gear = 0

        if control.planning_info.mode == "delivery1" or control.planning_info.mode == "delivery2":
            temp_msg.speed = 8
        else:
            temp_msg.speed=5
        temp_msg.brake = 0
        temp_msg.encoder = 0
        temp_msg.emergency_stop = 0
        if control.planning_info.mode == "small" or control.planning_info.mode == "big":
            temp_msg.path_steer=0
        else: 
            temp_msg.path_steer=control.local_path.heading[self.target_index]
        print(temp_msg.path_steer)
        temp_msg.auto_manual = 1
        target=PointCloud()
        target.points.append(self.target_point)
        target.header.frame_id='world'
        target.header.stamp=rospy.Time.now()
        self.target_pub.publish(target)

        return temp_msg

    