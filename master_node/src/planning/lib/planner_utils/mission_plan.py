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
            
        elif planner.surface_msg is "stopline" and self.serial_msg.speed > 10 and abs(self.srial_msg.steer) < 5:
            self.mode = 'normal_stop'
            self.mission_ing = True
            
        elif self.local.x is coordinate: # Dyanamic -- person
            self.mode = 'emergency_stop'
            self.mission_ing = True
        
        elif self.local.x is 3: # Static -- cone
            self.mode = 'avoidance'
            self.mission_ing = True

        elif 4 is 4: 
            self.mission_ing = True
        
        else:
            self.mode = 'general'


        return self.mode