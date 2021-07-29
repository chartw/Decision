from master_node.msg import Path
import csv
from math import degrees, hypot, radians, cos
from geometry_msgs.msg import Point32



class ParkingPlan:
    def __init__(self, planner):
        self.local = planner.local
        self.parking_path=Path()
        self.parking_target=0
        self.target_index = planner.target_index
        self.global_path = planner.parking_path


    def make_parking_path(self, target):
        self.parking_target = target
        with open("./map/kcity_map/Parking_kcity" + self.parking_target + ".csv", mode="r") as csv_file:
            csv_reader = csv.reader(csv_file)
            for line in csv_reader:
                self.parking_path.x.append(float(line[0]))
                self.parking_path.y.append(float(line[1]))
                self.parking_path.heading.append(degrees(float(line[2])))
                self.parking_path.k.append(float(line[3]))
                # self.parking_path.s.append(float(line[4]))
                # self.parking_path.etc.append(line[5])
                # self.parking_path.mission.append(line[6])


        return self.parking_path



    def point_plan(self, planner, lookahead):
        self.target_index = 0
        valid_idx_list = []
        idx = 0
        min_idx=0
        min_dist=-1
        if hypot(self.parking_path.x[self.target_index] - planner.local.x, self.parking_path.y[self.target_index] - planner.local.y) < lookahead:
            idx = self.target_index
        else:
            idx = 0
        for i in range(idx, len(self.parking_path.x)):
            dis = hypot(self.parking_path.x[i] - planner.local.x, self.parking_path.y[i] - planner.local.y)

            if dis < min_dist or min_dist == -1:
                min_dist=dis
                min_idx=i

            if dis <= lookahead:
                valid_idx_list.append(i)
            if len(valid_idx_list) != 0 and dis > lookahead:
                break
        if valid_idx_list:
            self.target_index = valid_idx_list[len(valid_idx_list) - 1]
        else:
            self.target_index=min(min_idx+40,len(self.parking_path.x)-1)

        if self.target_index==len(self.parking_path.x)-1:
            self.target_index=min_idx+40 - (len(self.parking_path.x)-1)

        theta = radians(planner.local.heading - self.parking_path.heading[self.target_index])
        proj_dist = lookahead * cos(radians(theta))
        planner.veh_index = max(0,self.target_index - int(proj_dist * 10))

        target_point=Point32(self.parking_path.x[self.target_index],self.parking_path.y[self.target_index],0)
        print(self.parking_path.heading[self.target_index])
        return self.target_index, target_point
