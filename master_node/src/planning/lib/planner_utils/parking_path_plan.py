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
        self.global_path = planner.parking_path
        self.parking_state = "parking_init"
        self.time_count = 0
        self.time_bcount= 0
        self.time_ecount= 0
        self.target_index=0


        self.base = []
        self.base.append(Point32(4.88, 8.876, 0))
        self.base.append(Point32(8.725, 16.832, 0))
        #self.base.append(Point32(9.612609543917143, 17.94208643988677, 0))

        self.start_base = self.base[0]

        self.parking_lot = []
        self.parking_lot.append(Point32(12.1308345505808, 11.0446013152771, 0))
        self.parking_lot.append(Point32(13.4108293420966, 13.6314223045673, 0))
        self.parking_lot.append(Point32(14.9418337803785, 16.2236247106905, 0))
        self.parking_lot.append(Point32(16.275, 18.867, 0))
        self.parking_lot.append(Point32(17.706, 21.457, 0))
        self.parking_lot.append(Point32(19.0818345505808, 24.1776013152771, 0))
        # self.parking_lot.append(Point32(17.623907356361915, 41.175622253568505, 0))
        # self.parking_lot.append(Point32(15.85266480396189, 38.844924089730185, 0))
        # self.parking_lot.append(Point32(14.16998329088652, 36.736197374027405, 0))
        # self.parking_lot.append(Point32(12.398738836681156, 34.405499957494584, 0))
        # self.parking_lot.append(Point32(10.716055698028432, 32.18578849457638, 0))

        planner.parking_target_index = 0

        self.veh_index = 0
        self.parking_target=-1


    def parking_state_decision(self, planner):
        print(planner.veh_index)
        if self.parking_state == "parking_init":
            
            print("======Initialize Parking")


            if hypot(planner.global_path.x[920]-planner.local.x, planner.global_path.y[920]-planner.local.y)<3:
                self.parking_state = "parking-base1"
                self.start_base = self.base[0]
                self.time_count=time()

        elif self.parking_state == "parking-base1":
            #TODO: ?????? ????????? 2??? ?????? ?????????
            print("======Waiting on Parking base 1")
            self.parking_target=planner.parking_target


            if time() - self.time_count > 3 and self.parking_target <= 3:
                self.parking_state = "parking_ready"
            elif time() - self.time_count > 3 and self.parking_target > 3:
                self.parking_state = "parking_going2next"
                self.time_count=time()
        
        elif self.parking_state == "parking_going2next":
            print("======Going to base2")
            # if time() - self.time_count > 4:
            if hypot(planner.global_path.x[1020]-planner.local.x, planner.global_path.y[1020]-planner.local.y)<2:
                self.time_count = time()
                self.parking_state = "parking-base2"

        elif self.parking_state == "parking-base2":
            print("======Waiting on Parking base 2")
            if time() - self.time_count > 3:
                self.parking_state = "parking_ready"

        elif self.parking_state == "parking_ready":

            if self.parking_target != -1:
                self.parking_state = "parking_start"
                print("Parking start for target ", self.parking_target)

        elif self.parking_state == "parking_start":
            print("======Parking for target ", self.parking_target)

            #????????? ???????????? ??????. 
            dis_x = self.parking_lot[self.parking_target - 1].x - planner.local.x
            dis_y = self.parking_lot[self.parking_target - 1].y - planner.local.y
            
            print("======Distance from goal_point: ", hypot(dis_x, dis_y))
            if hypot(dis_x, dis_y) < 2:
                self.parking_state = "parking_complete"
                self.time_count=time()
   

        elif self.parking_state == "parking_complete":
            if time() - self.time_count > 13:
                self.parking_state = "parking_backward"

        elif self.parking_state == "parking_backward":
            if hypot(planner.parking_backpath.x[-1]-planner.local.x, planner.parking_backpath.y[-1]-planner.local.y)<1.5:
                self.parking_state = "backward_complete"
                self.time_bcount=time()
                
        elif self.parking_state == "backward_complete":
            if time() - self.time_bcount > 4:
                self.parking_state = "parking_end"

        return self.parking_state


        

    def make_parking_path(self, parking_target):
        with open("./map/kcity_map/Parking_kcity/" + str(parking_target) + ".csv", mode="r") as csv_file:
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

    def point_plan(self,path,lookahead):
        max_index=len(path.x)-1
        min_idx=0
        min_dist=-1
        for i in range(max_index+1):
            dis = hypot(path.x[i] - self.local.x, path.y[i] - self.local.y)
            if dis < min_dist or min_dist == -1:
                min_dist=dis
                min_idx=i
        if min_dist > lookahead:
            self.target_index=min_idx
        else:
            self.target_index=int(min(min_idx+(lookahead-min_dist)*10,max_index))
        # print(self.target_index)
        target_point=Point32(path.x[self.target_index],path.y[self.target_index],0)
        return self.target_index, target_point

    '''
    def point_plan(self, planner, lookahead):
        valid_idx_list = []
        idx = 0
        min_idx=0
        min_dist=-1
        if hypot(self.parking_path.x[planner.parking_target_index] - planner.local.x, self.parking_path.y[planner.parking_target_index] - planner.local.y) < lookahead:
            idx = planner.parking_target_index
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
            planner.parking_target_index = valid_idx_list[len(valid_idx_list) - 1]
        else:
            planner.parking_target_index=min(min_idx+40,len(self.parking_path.x)-1)

        # if planner.parking_target_index==len(self.parking_path.x)-1:
        #     planner.parking_target_index=min_idx+40 - (len(self.parking_path.x)-1)

        theta = radians(planner.local.heading - self.parking_path.heading[planner.parking_target_index])
        proj_dist = lookahead * cos(radians(theta))
        self.veh_index = max(0,planner.parking_target_index - int(proj_dist * 10))

        target_point=Point32(self.parking_path.x[planner.parking_target_index],self.parking_path.y[planner.parking_target_index],0)
        # print(self.parking_path.heading[planner.parking_target_index])
        return planner.parking_target_index, target_point
    '''
