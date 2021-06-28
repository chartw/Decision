#-*- coding:utf-8 -*-

import rospy
import threading
from mission_planner import MissionPlanner
from control_planner import ControlPlanner

class Planner():
  def __init__(self):
    rospy.init_node('Planner', anonymous=False)

    self.planning_data = {
                  'mission': 'general', \
                  'cur_x': 0.0, \
                  'cur_y': 0.0, \
                  'cur_yaw': 0.0, \
                  'test': 0 \
                  }

    self.control_data = {
                  'steering': 0, \
                  'target_speed': 50, \
                  'break_val': 0, \
                  'test' : 0 \
                  }


    # Loop 1 by rospy.spin
    self.receiver = Receiver(self)
    th_receiver = threading.Thread(target=self.receiver.run, args=())
    th_receiver.start()
    print('===Receiver Start!')

    # Loop 2 by rospy.spin
    self.transmitter = Transmitter(self)
    th_transmitter = threading.Thread(target=self.transmitter.run, args=())
    th_transmitter.start()
    print('===Transmitter Start!')

    # Loop 3 by while
    self.mission_planner = MissionPlanner(self)
    print("====Mission Planner Start!")
    self.control_planner = ControlPlanner(self)
    print("====Control Planner Start!")

  def run(self):
    rate=rospy.Rate(100) # 100hz
    while not rospy.is_shutdown():
            self.mission_planner.run()
            self.control_planner.run()
            rate.sleep()



if __name__ == "__main__":
    planner = Planner()
    planner.run()