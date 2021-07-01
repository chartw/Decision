class MissonPlan:
    def __init__(self,planner):
        self.obstacles = planner.obstacles
        self.objects=planner.objects
        self.position=planner.position
        self.mode=planner.planning_msg.mode

    def decision(self):
        
        if len(self.obstacles.segments)!=0 and :

    def object_decision(self):