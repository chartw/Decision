from master_node.msg import Path
import csv
from math import degrees, hypot, radians, cos
from geometry_msgs.msg import Point32
from time import time


class ParkingPlan:
    def __init__(self, planner):
        self.local = planner.local
        self.parking_path=Path()
        self.parking_target=0
        self.target_index = planner.target_index
        self.global_path = planner.parking_path
        self.parking_state = "parking_init"
        self.time_count = 0

        self.base = []
        self.base.append(Point32(8.133715275780418, 14.982628122549599, 0))
        self.base.append(Point32(9.612609543917143, 17.94208643988677, 0))

        self.parking_lot = []
        self.parking_lot.append(Point32(17.623907356361915, 41.175622253568505, 0))
        self.parking_lot.append(Point32(15.85266480396189, 38.844924089730185, 0))
        self.parking_lot.append(Point32(14.16998329088652, 36.736197374027405, 0))
        self.parking_lot.append(Point32(12.398738836681156, 34.405499957494584, 0))
        self.parking_lot.append(Point32(10.716055698028432, 32.18578849457638, 0))




    def parking_state_decision(self, planner):
        if self.parking_state == "parking_init":
            
            print("======Initialize Parking")


            if hypot(self.base[0].x-self.local.x, self.base[0].y-self.local.y)<2:
                self.parking_state = "parking-base1"
                self.time_count=time.time()

        elif self.parking_state == "parking-base1":
            #TODO: 파킹 베이스 2로 가는 조건문

            if time.time() - self.time_count > 3:
                self.parking_state = "parking_ready"

        elif self.parking_state == "parking_ready":
            self.parking_target = planner.parking_target

            if self.parking_target != -1:
                self.parking_state = "parking_start"

        elif self.parking_state == "parking_start":

            #밖에서 파킹스택 쌓기. 


            dis_x = self.parking_lot[self.parking_target].x-planner.local.x
            dis_y = self.parking_lot[self.parking_target].y-planner.local.y

            if hypot(dis_x, dis_y) < 1:
                self.parking_state = "parking_complete"
                self.time_count=time.time()
   

        elif self.parking_state == "parking_complete":
            
            if time.time() - self.time_count > 3:
                self.parking_state = "parking_backward"

        elif self.parking_state == "parking_backward":
            스택에서 빼서 리턴?


        return self.parking_state

    def parking_base_1(self):
        
          

    #def parking_base_2(self):

    def parking_ready(self):
        self.parking_state = "parking-ready"

    def parking_start(self):
        
    def parking_complete(self):
    
    def parking_backward(self):
    
        

    def make_parking_path(self):
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
        # print(self.parking_path.heading[self.target_index])
        return self.target_index, target_point
