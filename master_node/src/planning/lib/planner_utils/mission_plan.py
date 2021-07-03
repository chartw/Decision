from math import hypot

class MissonPlan:
    def __init__(self,planner):
        self.obstacles = planner.obstacles
        self.objects=planner.objects
        self.local=planner.local
        self.surface_msg = planner.surface_msg
        self.serial_msg = planner.serial_msg
        self.mode=""
        self.mission_ing = planner.mission_ing # True / False



    def decision(self):  

        if 1 is 2: # Parking
            self.mode = 'parking'
            self.mission_ing = True
            
        elif self.surface_msg is "stopline" and self.serial_msg.speed > 10 and abs(self.srial_msg.steer) < 5:
            self.mode = 'normal_stop'
            self.mission_ing = True

        # Dyanamic -- person stop at node 24    
        elif hypot(self.local.x - 2.125, self.local.y - 43.617) < 1:
            self.mode = 'emergency_stop'
            self.mission_ing = True
            
        # Static -- cone avoidance at node 16
        elif hypot(self.local.x - 29.757, self.local.y - 35.737) < 1: 
            self.mode = 'avoidance'
            self.mission_ing = True

        elif 4 is 4: 
            self.mission_ing = True
        
        else:
            self.mode = 'general'


        return self.mode