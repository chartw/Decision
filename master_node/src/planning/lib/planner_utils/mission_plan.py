from geometry_msgs.msg import Point32
from math import hypot
import time
from lib.planner_utils.mapping import Mapping
from master_node.msg import Path



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
        self.base.append(Point32(4.88, 8.876, 0))
        self.base.append(Point32(8.725, 16.832, 0))
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
        mission=planner.global_path.mission[planner.veh_index]
        env=planner.global_path.env[planner.veh_index]
        dist=planner.planning_msg.dist

        # print(mission, env, dist)
        if mission=="small" or mission=="big":
            if dist >0:
                self.mode = mission
                planner.local_path_maker.start(planner)
            if len(planner.local_path.x)!=0:
                if hypot(planner.local_path.x[-1] - planner.local.x, planner.local_path.y[-1] - planner.local.y) < 3:
                    planner.local_path=Path()
                    self.mode="general"
        # print("???????????", hypot(self.base[0].x-self.local.x, self.base[0].y-self.local.y))

        if hypot(4.2-self.local.x, 7.7-self.local.y) < 2:
            self.mode = "parking"
            self.mission_ing = True

        elif mission=="kid":
            if env=="bump":
                self.mode="bump"
            elif dist > 0 and dist < 5:
                self.mode="emergency_stop"
            else:
                self.mode="kid"

        else:
            self.mode="general"

        return self.mode

        # 주차 공간이 무조건 하나 있다고 생각했을때의 parking mode들.
        # 만약 주차공간이 없을경우, 그냥 지나치는것도 가정할거면 base2이후의 모드를 더 추가해야 함
        # parking이고, base1에 가까이 올경우 -> parking-base1으로 변경하고 정지하여 그때의 시간 측정
        elif mode =='parking' and hypot(self.base[0].x-self.local.x, self.base[0].y-self.local.y)<1:
            mode='parking-base1'
            self.time_count=time.time()

        # parking2이고(base1에서 주차공간 찾지 못함), base2에 가까이 올 경우 -> parking-base2로 변경하고 정지하여 그때의 시간 측정
        elif mode =='parking2' and hypot(self.base[1].x-self.local.x, self.base[1].y-self.local.y)<1:
            mode='parking-base2'
            self.time_count=time.time()

        # base에 정지해 있는 시간이 일정 시간 지날경우 -> parking-ready 로 변경. 이때 LiDAR로부터 주차 공간 수신
        elif (mode =='parking-base1' or mode=='parking-base2') and self.time_count- time.time() > 3:
            mode='parking-ready'

        # parking-ready 일때, 유효한 주차공간이 들어올 경우 -> parking-start로 변경. 이때의 heading값 임시 저장. 주차 주행 시작
        elif mode=='parking-ready':
            if self.parking_msg!=-1:
                mode='parking-start'
                self.temp_heading=self.local.heading

            # 유효한 주차공간이 들어오지 않을 경우 -> parking2로 변경하여 base2를 향해 주행
            elif self.parking_msg==-1:
                mode=='parking2'

        # parking-start일때, 주차 공간 중점과 가까워지면 -> parking-complete로 변경. 이때의 시간 측정하여 일정시간 정지. 
        elif mode=='parking-start' and hypot(self.parking_lot[self.parking_msg].x-self.local.x,self.parking_lot[self.parking_msg].y-self.local.y) < 1:
            mode='parking-complete'
            self.time_count=time.time()

        elif mode=='parking-complete' and self.time_count- time.time() > 3:
            mode='backward-start'

        elif mode=='backward-start' and abs(self.local.heading - self.temp_heading) < 5:
            mode='general'

        """

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

        return self.mode, self.mission_ing

    def end_check(self, planner):
        if planner.planning_msg.mode == "normal_stop":
            return self.serial_msg.speed > 0.01

        elif planner.planning_msg.mode == "avoidance" and len(planner.local_path.x)!=0:
            #print(hypot(planner.local_path.x[-1] - planner.local.x, planner.local_path.y[-1] - planner.local.y))
            #print("이거다시팚", hypot(planner.local_path.x[-1] - planner.local.x, planner.local_path.y[-1] - planner.local.y) > 3)
            if  hypot(planner.local_path.x[-1] - planner.local.x, planner.local_path.y[-1] - planner.local.y) > 3:
                return True
            else:
                return False

