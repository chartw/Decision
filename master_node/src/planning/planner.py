# -*- coding:utf-8 -*-

import rospy
import threading
from mission_planner import MissionPlanner
from control_planner import ControlPlanner
from communicator import Communicator


class Planner:
    def __init__(self):
        rospy.init_node("Planner", anonymous=False)

        self.planning_data = {
            "mission": "general",
            "cur_x": 0.0,
            "cur_y": 0.0,
            "cur_yaw": 0.0,
            "test": 0,
        }

        self.control_data = {
            "steering": 0,
            "target_speed": 50,
            "break_val": 0,
            "test": 0,
        }

        # 임시 데이터
        self.object_data = Obstacles()
        self.obstacle_data = BoundingBoxes()

        # Loop 1 by while
        self.communicator = Communicator(self)
        th_communicator = threading.Thread(target=self.communicator.run, args=())
        th_communicator.start()
        print("===Communicator Start!")

        # Loop 2 by while
        self.mission_planner = MissionPlanner(self)
        print("====Mission Planner Start!")
        self.control_planner = ControlPlanner(self)
        print("====Control Planner Start!")

    def run(self):
        rate = rospy.Rate(100)  # 100hz
        while not rospy.is_shutdown():
            self.mission_planner.run()
            self.control_planner.run()
            rate.sleep()


if __name__ == "__main__":
    planner = Planner()
    planner.run()
