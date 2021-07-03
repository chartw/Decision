from geometry_msgs.msg import Point32
from math import hypot

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



    def decision(self, planner):  
        
        
        if  planner.gpp_requested: # Parking
            self.mode = 'parking'

        elif self.mode =='parking' and hypot(self.base[0].x-self.local.x, self.base[0].y-self.local.y)<1:
            self.mode='parking-base1'

        elif (self.mode =='parking-base1' or self.mode=='parking-base2') and time_count > 3:
            self.mode='parking-ready'

        elif self.mode=='parking-ready' and self.parking_msg!=-1:
            self.mode='parking-start'

        elif self.mode=='parking-ready' and self.parking_msg==-1:
            self.mode=='parking'

        elif self.mode=='parking-start' and hypot(self.parking_lot[self.parking_msg].x-self.local.x,self.parking_lot[self.parking_msg].y-self.local.y) < 1:
            self.mode='parking-complete'
        





        elif self.mode =='parking-base1' and self.parking_msg==-1 and self.calc_dis(self.base2.x, self.base2.y) < 1:
            self.mode='parking-base2'

     
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