from geometry_msgs.msg import Point32
from math import hypot
import time

class MissonPlan:
    def __init__(self,planner, base, parking_lot):
        self.obstacles = planner.obstacles
        self.objects=planner.objects
        self.local=planner.local
        self.surface_msg = planner.surface_msg
        self.serial_msg = planner.serial_msg
        self.parking_msg=planner.parking_msg
        self.mode=planner.planning_msg.mode
        self.mission_ing = planner.mission_ing # True / False

        self.base=[]
        self.base.append(Point32(22.760400877965,41.7303388307402,0))
        self.base.append(Point32(17.978170358155626,34.84945192598553,0))

        self.parking_lot=[]
        self.parking_lot.append(Point32(17.623907356361915,41.175622253568505,0))
        self.parking_lot.append(Point32(15.85266480396189,38.844924089730185,0))
        self.parking_lot.append(Point32(14.16998329088652,36.736197374027405,0))
        self.parking_lot.append(Point32(12.398738836681156,34.405499957494584,0))
        self.parking_lot.append(Point32(10.716055698028432,32.18578849457638,0))

        self.time_count=0
        self.temp_heading=0


    def decision(self, planner):  
        
        
        if not planner.gpp_requested: # Parking
            self.mode = 'parking'

        # 주차 공간이 무조건 하나 있다고 생각했을때의 parking mode들.
        # 만약 주차공간이 없을경우, 그냥 지나치는것도 가정할거면 base2이후의 모드를 더 추가해야 함
        # parking이고, base1에 가까이 올경우 -> parking-base1으로 변경하고 정지하여 그때의 시간 측정
        elif self.mode =='parking' and hypot(self.base[0].x-self.local.x, self.base[0].y-self.local.y)<1:
            self.mode='parking-base1'
            self.time_count=time.time()

        # parking2이고(base1에서 주차공간 찾지 못함), base2에 가까이 올 경우 -> parking-base2로 변경하고 정지하여 그때의 시간 측정
        elif self.mode =='parking2' and hypot(self.base[1].x-self.local.x, self.base[1].y-self.local.y)<1:
            self.mode='parking-base2'
            self.time_count=time.time()

        # base에 정지해 있는 시간이 일정 시간 지날경우 -> parking-ready 로 변경. 이때 LiDAR로부터 주차 공간 수신
        elif (self.mode =='parking-base1' or self.mode=='parking-base2') and self.time_count- time.time() > 3:
            self.mode='parking-ready'

        # parking-ready 일때, 유효한 주차공간이 들어올 경우 -> parking-start로 변경. 이때의 heading값 임시 저장. 주차 주행 시작
        elif self.mode=='parking-ready':
            if self.parking_msg!=-1:
                self.mode='parking-start'
                self.temp_heading=self.local.heading

            # 유효한 주차공간이 들어오지 않을 경우 -> parking2로 변경하여 base2를 향해 주행
            elif self.parking_msg==-1:
                self.mode=='parking2'

        # parking-start일때, 주차 공간 중점과 가까워지면 -> parking-complete로 변경. 이때의 시간 측정하여 일정시간 정지. 
        elif self.mode=='parking-start' and hypot(self.parking_lot[self.parking_msg].x-self.local.x,self.parking_lot[self.parking_msg].y-self.local.y) < 1:
            self.mode='parking-complete'
            self.time_count=time.time()

        elif self.mode=='parking-complete' and self.time_count- time.time() > 3:
            self.mode='backward-start'

        elif self.mode=='backward-start' and abs(self.local.heading - self.temp_heading) < 5:
            self.mode='general'

        






        elif self.mode=='avoidance' and hypot(planner.mission_goal.x-self.local.x,planner.mission_goal.y-self.local.y) < 1:
            self.mode='general'
        

     
        # elif planner.surface_msg is "stopline" and self.serial_msg.speed > 10 and abs(self.srial_msg.steer) < 5:
        #     self.mode = 'normal_stop'
        #     self.mission_ing = True
            
        # elif self.local.x is coordinate: # Dyanamic -- person
        #     self.mode = 'emergency_stop'
        #     self.mission_ing = True
        
        # elif self.local.x is 3: # Static -- cone
        #     self.mode = 'avoidance'
        #     self.mission_ing = True

        # elif 4 is 4: 
        #     self.mission_ing = True
        
        # else:
        #     self.mode = 'general'


        return self.mode

    def calc_dis(self, nx, ny):
        # print(nx, ny, )
        distance = ((nx - self.local.x)**2 +  (ny - self.local.y)**2)**0.5

        return distance