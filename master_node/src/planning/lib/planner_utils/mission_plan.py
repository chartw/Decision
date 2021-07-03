class MissonPlan:
    def __init__(self,planner):
        self.obstacles = planner.obstacles
        self.objects=planner.objects
        self.position=planner.position
        self.surface_msg = planner.surface_msg
        self.serial_msg = planner.serial_msg
        self.mode=""

    def decision(self):
        
        
        if 1 is 2:
            pass
        
        elif planner.surface_msg is "stopline" and self.serial_msg.speed > 10 and abs(self.srial_msg.steer) < 5:
            self.mode = 'normal_stop'
            
        elif 2 is 2:
            pass
        
        elif 3 is 3:
            pass
        
        else:
            self.mode = 'general'
       


        return self.mode