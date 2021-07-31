from geometry_msgs.msg import Point32
from math import hypot
import time
from lib.planner_utils.mapping import Mapping



class MissionPlan:
    def __init__(self, planner):

        self.obstacle_msg = planner.obstacle_msg
        self.object_msg = planner.object_msg
        self.local = planner.local
        self.surface_msg = planner.surface_msg
        self.serial_msg = planner.serial_msg
        self.parking_msg = planner.parking_msg
        self.object_msg = planner.object_msg
        self.mission_ing = planner.mission_ing  # True / False
        self.base = []
        self.base.append(Point32(8.133715275780418, 14.982628122549599, 0))
        self.base.append(Point32(9.612609543917143, 17.94208643988677, 0))
        #self.base.append(Point32(22.760400877965, 41.7303388307402, 0))
        #self.base.append(Point32(17.978170358155626, 34.84945192598553, 0))
        self.mode=""
        self.pmode=""

        self.parking_lot = []
        self.parking_lot.append(Point32(17.623907356361915, 41.175622253568505, 0))
        self.parking_lot.append(Point32(15.85266480396189, 38.844924089730185, 0))
        self.parking_lot.append(Point32(14.16998329088652, 36.736197374027405, 0))
        self.parking_lot.append(Point32(12.398738836681156, 34.405499957494584, 0))
        self.parking_lot.append(Point32(10.716055698028432, 32.18578849457638, 0))

        self.time_count = 0
        self.temp_heading = 0

        self.start_time = time.time()
        self.current_time = time.time()

    def decision(self, planner):


        # print("???????????", hypot(self.base[0].x-self.local.x, self.base[0].y-self.local.y))

        if hypot(4.2-self.local.x, 7.7-self.local.y) < 9:
            self.mode = "parking"

        elif planner.object_msg.data == "normal_stop":
            self.mode = "normal_stop"
            self.mission_ing = True

        elif planner.object_msg.data == "avoid":
            self.mode = "avoidance"
            self.mission_ing = True
            planner.is_avoidance_ing=False
            
        elif self.mode == "general"and planner.dynamic_flag == False and planner.planning_msg.dist!=-1:
            planner.local_path_maker.start(planner)
            self.mode = "avoidance"
            self.mission_ing = True
            planner.is_avoidance_ing=False
            planner.planning_msg.dist=-1


        # Parking
        if self.mode == "parking":
            if hypot(self.base[0].x-self.local.x, self.base[0].y-self.local.y)<2:
                self.pmode = "parking-base1"
                self.time_count=time.time()
                print("resetttttttttttttttttttttttttt")

            elif (self.pmode == "parking-base1" or self.pmode == "parking-base2") and time.time() - self.time_count > 3:
                self.pmode = "parking-ready"
            
            elif self.pmode=='parking-ready':
                if self.parking_msg!=-1:
                    self.pmode='parking-start'
                    self.temp_heading=self.local.heading

                # 유효한 주차공간이 들어오지 않을 경우 -> parking2로 변경하여 base2를 향해 주행
                elif self.parking_msg==-1:
                    self.pmode == 'parking2'

            # 실제 주차 프로세스
            elif self.pmode == "parking-start" and hypot(self.parking_lot[self.parking_msg].x-self.local.x, self.parking_lot[self.parking_msg].y-self.local.y) < 1:
                self.pmode = "parking-complete"
            
            elif self.pmode == "parking-complete":
                self.pmode = "backward-start"

            elif self.pmode == "backward-start" and abs(self.local.heading - self.temp_heading) < 5:
                self.pmode = 'general'

            else:
                self.mode = "general"
                self.mission_ing=False


        if self.mission_mode == "Backward":
            print('@@@@@@@@@백월드@@@@@@@@@')
            parking_time_end=time.time() + 6.5
            while time.time() < parking_time_end:
                
                cnt=0x00
                result = self.ser.readline() 
                self.ser.flushInput()
                # print(result)
                # print(result[0])
                if (result[0] is 0x53 and result[1] is 0x54 and result[2] is 0x58):
                    res_arr = []
                    res_idx = 0
                    # print('okokok')

                    while True:
                        for i in range(len(result)):
                            if result[i] is 0x0A and i is not 17:
                                # print("### 0x0A Found!", i, "th data")
                                res_arr.append(0x0B)
                            else :
                                res_arr.append(result[i])

                        if len(res_arr) < 18:
                            result = self.ser.readline()
                        els
        

        # elif planner.surface_msg is "stopline" and self.serial_msg.speed > 10 and abs(self.srial_msg.steer) < 5:
        #     mode = 'normal_stop'
        #     self.mission_ing = True

        # elif self.surface_msg is "stopline" and self.serial_msg.speed > 10 and abs(self.srial_msg.steer) < 5:
        #     return "normal_stop"

        # Dyanamic -- person stop at node 24
        # elif hypot(self.local.x - 2.125, self.local.y - 43.617) < 1:
        #     mode = 'emergency_stop'
        #     self.mission_ing = True

        # # Static -- cone avoidance at node 16
        # elif hypot(self.local.x - 29.757, self.local.y - 35.737) < 1:
        #     mode = 'avoidance'
        #     self.mission_ing = True
        # elif self.local.x is coordinate: # Dyanamic -- person
        #     mode = 'emergency_stop'
        #     self.mission_ing = True

        # elif self.local.x is 3: # Static -- cone
        #     mode = 'avoidance'
        #     self.mission_ing = True

        # elif 4 is 4:
        #     self.mission_ing = True

        # print("in the mission_plan.py", self.mode, self.pmode)
        return self.mode, self.mission_ing, self.pmode

    def end_check(self, planner):
        if planner.planning_msg.mode == "normal_stop":
            return self.serial_msg.speed > 0.01

        elif planner.planning_msg.mode == "avoidance" and len(planner.local_path.x)!=0:
            #print(hypot(planner.local_path.x[-1] - planner.local.x, planner.local_path.y[-1] - planner.local.y))
            if  hypot(planner.local_path.x[-1] - planner.local.x, planner.local_path.y[-1] - planner.local.y) > 3:
                return True
            else:
                return False

