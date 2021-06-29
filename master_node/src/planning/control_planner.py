#-*- coding:utf-8 -*-
import rospy
from missions.parking import Parking

class ControlPlanner():
    def __init__(self, planner):
        self.planning_data = planner.planning_data
        self.control_data = planner.control_data

    def run(self):
      self.mission_name = "mission_" + self.planning_data['mission']
      self.mission = getattr(self, self.mission_name, lambda:"general")
      return self.mission() # Switch문 비슷하게 동작함.
    


    def mission_general(self):
        print("ControlPlanner: mission general")



    def mission_parking(self):
        print("ControlPlanner: mission parking start")
        p = Parking(self)
        p.parking_start()
        p.parking_complete()
        p.backward_start()
        p.backward_complete()
        p.driving_start()
        print("ControlPlanner: mission parking complete")

        self.planner_data['mission'] = 'general'


    def mission_avoidance(self):
        print("ControlPlanner: mission avoidance")




    def mission_stop(self):
        print("ControlPlanner: mission stop")



    def mission_uturn(self):
        print("ControlPlanner: mission uturn")