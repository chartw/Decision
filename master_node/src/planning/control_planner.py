#-*- coding:utf-8 -*-

import rospy


class ControlPlanner():
    def __init__(self, planner):
        self.data = planner.planning_data

    def run(self):
      self.mission_name = "mission_" + self.data['mission']
      self.mission = getattr(self, self.mission_name, lambda:"general")
      return self.mission()
    # Switch문 비슷하게 동작함.


    def mission_general(self):
        print("ControlPlanner: mission general")




    def mission_parking(self):
        print("ControlPlanner: mission parking")





    def mission_avoidance(self):
        print("ControlPlanner: mission avoidance")




    def mission_stop(self):
        print("ControlPlanner: mission stop")




    def mission_uturn(self):
        print("ControlPlanner: mission uturn")
        
